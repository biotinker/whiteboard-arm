package main

import (
	"context"
	"fmt"
	"github.com/golang/geo/r3"
	"math"
	"strconv"
	"strings"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	//~ pb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
	"go.viam.com/utils/rpc"

	servicepb "go.viam.com/api/service/motion/v1"
	"go.viam.com/rdk/services/motion"
)

const (
	cloudUrl = ""
	secret   = ""

	zAdj = 3         // adjustment to writing altitude
	zBuf = zAdj + 20 // adjustment to non-writing holding height

	eraserBuf = zAdj + 3 // eraser held this much further away than marker

	armName = "xArm7"

	markerLen = 170.
	eraserLen = 180.
	eraserX   = 110.
	eraserY   = 55.

	// 7-segment line length and spacing
	segLen    = 100.
	segBuffer = 10.

	digitXoffset = 150.
	colonWidth   = 55.
)

var (
	// Measured points on glass plane
	pt1 = r3.Vector{-479.8, -410.3, 613.5}
	pt2 = r3.Vector{-62.4, -398.2, -147.4}
	pt3 = r3.Vector{216.1, -400, 734.4}

	home7 = referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0, 0})

	v1    = pt2.Sub(pt1)
	v2    = pt3.Sub(pt1)
	pNorm = v1.Cross(v2).Normalize()

	allowWrite = []*servicepb.CollisionSpecification_AllowedFrameCollisions{
		// BUG workaround
		&servicepb.CollisionSpecification_AllowedFrameCollisions{Frame1: "marker_origin", Frame2: "glass"},
	}
	allowErase = []*servicepb.CollisionSpecification_AllowedFrameCollisions{
		// BUG workaround
		&servicepb.CollisionSpecification_AllowedFrameCollisions{Frame1: "eraser_origin", Frame2: "glass"},
	}

	linearConstraint = &servicepb.Constraints{LinearConstraint: []*servicepb.LinearConstraint{&servicepb.LinearConstraint{}}}
	writeConstraint  = &servicepb.Constraints{
		LinearConstraint:       []*servicepb.LinearConstraint{&servicepb.LinearConstraint{}},
		CollisionSpecification: []*servicepb.CollisionSpecification{&servicepb.CollisionSpecification{Allows: allowWrite}},
	}
	eraseConstraint = &servicepb.Constraints{
		LinearConstraint:       []*servicepb.LinearConstraint{&servicepb.LinearConstraint{}},
		CollisionSpecification: []*servicepb.CollisionSpecification{&servicepb.CollisionSpecification{Allows: allowErase}},
	}
)

// This will test solving the path to write the word "VIAM" on a whiteboard.
func GenerateTransforms() []*referenceframe.LinkInFrame {

	if pNorm.Y < 0 {
		pNorm = pNorm.Mul(-1)
	}

	glassFrame := referenceframe.NewLinkInFrame(
		"world",
		spatialmath.NewPose(pt3, &spatialmath.OrientationVectorDegrees{Theta: 90, OX: pNorm.X, OY: pNorm.Y, OZ: pNorm.Z}),
		"glass",
		nil,
	)

	markerOriginFrame := referenceframe.NewLinkInFrame(
		armName,
		spatialmath.NewPoseFromOrientation(&spatialmath.OrientationVectorDegrees{OY: -1, OZ: 1}),
		"marker_base",
		nil,
	)
	markerGeom, _ := spatialmath.NewCapsule(spatialmath.NewPoseFromPoint(r3.Vector{0, 0, -markerLen / 2}), 15, markerLen, "marker")
	markerFrame := referenceframe.NewLinkInFrame(
		"marker_base",
		spatialmath.NewPoseFromPoint(r3.Vector{0, 0, markerLen}),
		"marker",
		markerGeom,
	)

	eraserOriginFrame := referenceframe.NewLinkInFrame(
		armName,
		spatialmath.NewPoseFromOrientation(&spatialmath.OrientationVectorDegrees{OY: 1, OZ: 1}),
		"eraser_base",
		nil,
	)
	eraserStemGeom, _ := spatialmath.NewCapsule(spatialmath.NewPoseFromPoint(r3.Vector{0, 0, -eraserLen / 2}), 15, markerLen, "eraser_stem")
	eraserStemFrame := referenceframe.NewLinkInFrame(
		"eraser_base",
		spatialmath.NewPoseFromPoint(r3.Vector{0, 0, eraserLen}),
		"eraser_stem",
		eraserStemGeom,
	)
	eraserGeom, _ := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{eraserX, eraserY, 0}, "eraser")
	eraserFrame := referenceframe.NewLinkInFrame(
		"eraser_stem",
		spatialmath.NewZeroPose(),
		"eraser",
		eraserGeom,
	)

	transforms := []*referenceframe.LinkInFrame{
		markerOriginFrame,
		markerFrame,
		eraserOriginFrame,
		eraserStemFrame,
		eraserFrame,
		glassFrame,
	}
	return transforms
}

func GenerateObstacles() []*referenceframe.GeometriesInFrame {

	obstaclesInFrame := []*referenceframe.GeometriesInFrame{}

	// Add the table obstacle to a WorldState
	obstacles := make([]spatialmath.Geometry, 0)

	leftSideOrigin := spatialmath.NewPoseFromPoint(r3.Vector{351.1, -399.2, 734.4})
	leftSideDims := r3.Vector{X: 50.0, Y: 100.0, Z: 5000.0}
	leftSideObj, _ := spatialmath.NewBox(leftSideOrigin, leftSideDims, "leftSide")
	obstacles = append(obstacles, leftSideObj)

	rightSideOrigin := spatialmath.NewPoseFromPoint(r3.Vector{-649.8, -412.3, 613.5})
	rightSideDims := r3.Vector{X: 200.0, Y: 300.0, Z: 5000.0}
	rightSideObj, _ := spatialmath.NewBox(rightSideOrigin, rightSideDims, "rightSide")
	obstacles = append(obstacles, rightSideObj)

	standOrigin := spatialmath.NewPoseFromPoint(r3.Vector{X: 0.0, Y: 0.0, Z: -250.0})
	standDims := r3.Vector{X: 200.0, Y: 200.0, Z: 500.0}
	standObj, _ := spatialmath.NewBox(standOrigin, standDims, "stand")
	obstacles = append(obstacles, standObj)

	obstaclesInFrame = append(obstaclesInFrame, referenceframe.NewGeometriesInFrame(referenceframe.World, obstacles))

	// Add the glass as a geometry
	glassDims := r3.Vector{X: 20000.0, Y: 20000.0, Z: 0.0}
	glassObj, _ := spatialmath.NewBox(spatialmath.NewZeroPose(), glassDims, "glass")

	obstaclesInFrame = append(obstaclesInFrame, referenceframe.NewGeometriesInFrame("glass", []spatialmath.Geometry{glassObj}))

	return obstaclesInFrame
}

func planeDist(query r3.Vector) float64 {

	// get the constant value for the plane
	pConst := -query.Dot(pNorm)

	return math.Abs(pNorm.Dot(query) + pConst)
}

func main() {
	logger := golog.NewDevelopmentLogger("client")
	robot, err := client.New(
		context.Background(),
		cloudUrl,
		logger,
		client.WithDialOptions(rpc.WithCredentials(rpc.Credentials{
			Type:    utils.CredentialsTypeRobotLocationSecret,
			Payload: secret,
		})),
	)
	if err != nil {
		logger.Fatal(err)
	}

	motionService, err := motion.FromRobot(robot, "builtin")
	if err != nil {
		logger.Fatal(err)
	}

	fmt.Println(planeDist(r3.Vector{216.1, -499.2, 734.4}))

	obstacles := GenerateObstacles()
	transforms := GenerateTransforms()

	worldState := &referenceframe.WorldState{
		Obstacles:  obstacles,
		Transforms: transforms,
	}
	_ = worldState
	defer robot.Close(context.Background())
	logger.Info("Resources:")
	logger.Info(robot.ResourceNames())

	markerResource := resource.Name{Name: "marker"}

	markerPoseInGlass, err := motionService.GetPose(context.Background(), markerResource, "glass", transforms, nil)
	if err != nil {
		fmt.Println(err)
	}
	fmt.Println(spatialmath.PoseToProtobuf(markerPoseInGlass.Pose()))

	// Move to start position
	goal := referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(r3.Vector{0, 0, 200}, &spatialmath.OrientationVectorDegrees{OZ: -1}))
	_, err = motionService.Move(context.Background(), markerResource, goal, worldState, nil, nil)
	if err != nil {
		fmt.Println(err)
	}
	//~ writeViam(motionService, worldState)
	writeTime(motionService, worldState)
}

func writeViam(motionService motion.Service, worldState *referenceframe.WorldState) {

	markerResource := resource.Name{Name: "marker"}

	for _, viamLetterPts := range viamPoints {
		startPt := viamLetterPts[0]
		startPt.Z += zBuf
		goal := referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(startPt, &spatialmath.OrientationVectorDegrees{OZ: -1}))
		_, err := motionService.Move(context.Background(), markerResource, goal, worldState, nil, nil)
		if err != nil {
			fmt.Println(err)
		}
		for _, viamPt := range viamLetterPts {
			adjPt := viamPt
			adjPt.Z += zAdj
			goal = referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(adjPt, &spatialmath.OrientationVectorDegrees{OZ: -1}))
			_, err = motionService.Move(context.Background(), markerResource, goal, worldState, writeConstraint, nil)
		}
		endPt := viamLetterPts[len(viamLetterPts)-1]
		endPt.Z += zBuf
		goal = referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(endPt, &spatialmath.OrientationVectorDegrees{OZ: -1}))
		_, err = motionService.Move(context.Background(), markerResource, goal, worldState, writeConstraint, nil)
	}

	// Move to final position
	goal := referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(r3.Vector{100, -100, 300}, &spatialmath.OrientationVectorDegrees{OZ: -1}))
	_, err := motionService.Move(context.Background(), markerResource, goal, worldState, nil, nil)
	if err != nil {
		fmt.Println(err)
	}
}

func writeTime(motionService motion.Service, worldState *referenceframe.WorldState) {

	oldHour := []string{".", "."}
	oldMinute := []string{".", "."}
	for {
		nowtime := time.Now()
		curHour := nowtime.Hour()
		curMinute := nowtime.Minute()

		// Write the starting time
		hourStr := strconv.Itoa(curHour)
		minuteStr := strconv.Itoa(curMinute)
		fmt.Println("hour", hourStr, "minute", minuteStr)
		if len(hourStr) == 1 {
			hourStr = "." + hourStr
		}
		if len(minuteStr) == 1 {
			minuteStr = "0" + minuteStr
		}
		for i, d := range hourStr {
			changeDigit(
				motionService,
				worldState,
				digitSegmentMap[oldHour[i]],
				digitSegmentMap[string(d)],
				r3.Vector{float64(i) * digitXoffset, 0, 0},
			)
		}
		for i, d := range minuteStr {
			changeDigit(
				motionService,
				worldState,
				digitSegmentMap[oldMinute[i]],
				digitSegmentMap[string(d)],
				r3.Vector{2*digitXoffset + colonWidth + float64(i)*digitXoffset, 0, 0},
			)
		}
		oldHour = strings.Split(hourStr, "")
		oldMinute = strings.Split(minuteStr, "")
		time.Sleep(5 * time.Second)
	}
}

func changeDigit(motionService motion.Service, worldState *referenceframe.WorldState, oldSegs, newSegs string, offset r3.Vector) {

	toErase := map[string]bool{}
	toWrite := map[string]bool{}
	for _, d := range oldSegs {
		toErase[string(d)] = true
	}
	for _, d := range newSegs {
		if _, ok := toErase[string(d)]; ok {
			// If here, then segment is already drawn
			toErase[string(d)] = false
		} else {
			// If here, then segment needs to be drawn
			toWrite[string(d)] = true
		}
	}
	fmt.Println("to erase", toErase)
	fmt.Println("to write", toWrite)

	// First erase unneeded segments
	for k, v := range toErase {
		if v {
			eraseSegment(motionService, worldState, k, offset)
		}
	}
	// now write new segments
	for k, v := range toWrite {
		if v {
			writeSegment(motionService, worldState, k, offset)
		}
	}
}

func writeSegment(motionService motion.Service, worldState *referenceframe.WorldState, segStr string, offset r3.Vector) {
	markerResource := resource.Name{Name: "marker"}
	segPts := segments[segStr]
	markerOrient := &spatialmath.OrientationVectorDegrees{OZ: -1, Theta:90}

	markerStart := segPts[0].Add(offset)
	markerStart.Z += zBuf
	markerPt1 := segPts[0].Add(offset)
	markerPt1.Z += zAdj
	markerPt2 := segPts[1].Add(offset)
	markerPt2.Z += zAdj
	markerEnd := segPts[1].Add(offset)
	markerEnd.Z += zBuf

	// move to just off the first point
	goal := referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(markerStart, markerOrient))
	_, err := motionService.Move(context.Background(), markerResource, goal, worldState, nil, nil)
	if err != nil {
		fmt.Println(err)
	}
	// touch glass
	goal = referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(markerPt1, markerOrient))
	_, err = motionService.Move(context.Background(), markerResource, goal, worldState, writeConstraint, nil)
	if err != nil {
		fmt.Println(err)
	}
	// write
	goal = referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(markerPt2, markerOrient))
	_, err = motionService.Move(context.Background(), markerResource, goal, worldState, writeConstraint, nil)
	if err != nil {
		fmt.Println(err)
	}
	// retreat from glass
	goal = referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(markerEnd, markerOrient))
	_, err = motionService.Move(context.Background(), markerResource, goal, worldState, writeConstraint, nil)
	if err != nil {
		fmt.Println(err)
	}
}

func eraseSegment(motionService motion.Service, worldState *referenceframe.WorldState, segStr string, offset r3.Vector) {
	eraserResource := resource.Name{Name: "eraser"}
	eraserOrient := &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 90}
	segPts := segments[segStr]

	var eraserPt1 r3.Vector
	var eraserPt2 r3.Vector

	// Determine start/end points from eraser and line dimensions
	if segPts[0].X == segPts[1].X {
		// vertical line
		eraserPt1 = segPts[0]
		eraserPt1.Y -= (eraserY/2 + segBuffer/2)
		eraserPt2 = segPts[1]
		eraserPt2.Y += (eraserY/2 + segBuffer/2)
	} else {
		//horizontal line
		eraserOrient = &spatialmath.OrientationVectorDegrees{OZ: -1}
		eraserPt1 = segPts[0]
		eraserPt1.X -= (eraserX/2 + segBuffer/2)
		eraserPt2 = segPts[1]
		eraserPt2.X += (eraserX/2 + segBuffer/2)
	}

	eraserPt1.Z = eraserBuf
	eraserStart := eraserPt1
	eraserStart.Z += zBuf
	eraserPt2.Z = eraserBuf
	eraserEnd := eraserPt2
	eraserEnd.Z += zBuf

	// move to just off the first point
	goal := referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(eraserStart.Add(offset), eraserOrient))
	_, err := motionService.Move(context.Background(), eraserResource, goal, worldState, nil, nil)
	if err != nil {
		fmt.Println(err)
	}
	// touch glass
	goal = referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(eraserPt1.Add(offset), eraserOrient))
	_, err = motionService.Move(context.Background(), eraserResource, goal, worldState, eraseConstraint, nil)
	if err != nil {
		fmt.Println(err)
	}
	// erase
	goal = referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(eraserPt2.Add(offset), eraserOrient))
	_, err = motionService.Move(context.Background(), eraserResource, goal, worldState, eraseConstraint, nil)
	if err != nil {
		fmt.Println(err)
	}
	// retreat from glass
	goal = referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(eraserEnd.Add(offset), eraserOrient))
	_, err = motionService.Move(context.Background(), eraserResource, goal, worldState, linearConstraint, nil)
	if err != nil {
		fmt.Println(err)
	}
}

// Write out the word "VIAM" backwards (visible through glass)
var viamPoints = [][]r3.Vector{
	// M
	[]r3.Vector{
		{220, -100, 0},
		{250, 0, 0},
		{280, -100, 0},
		{310, 0, 0},
		{340, -100, 0},
	},

	// A
	[]r3.Vector{
		{120, -100, 0},
		{160, 0, 0},
		{200, -100, 0},
	},
	// Middle bar of A
	[]r3.Vector{
		{140, -50, 0},
		{180, -50, 0},
	},

	// I
	[]r3.Vector{
		{100, 0, 0},
		{100, -100, 0},
	},

	// V
	[]r3.Vector{
		{0, 0, 0},
		{40, -100, 0},
		{80, 0, 0},
	},
}

//  _     A
// |_|  F   B
// |_|    G
//      E   C
//        D

var segments = map[string][]r3.Vector{
	"F": []r3.Vector{
		{0, -segBuffer, 0},
		{0, 0 - segBuffer - segLen, 0},
	},
	"E": []r3.Vector{
		{0, -segLen - 3*segBuffer, 0},
		{0, -segLen*2 - 3*segBuffer, 0},
	},
	"A": []r3.Vector{
		{segBuffer, 0, 0},
		{segBuffer + segLen, 0, 0},
	},
	"G": []r3.Vector{
		{segBuffer, -segLen - 2*segBuffer, 0},
		{segBuffer + segLen, -segLen - 2*segBuffer, 0},
	},
	"D": []r3.Vector{
		{segBuffer, -segLen*2 - 4*segBuffer, 0},
		{segBuffer + segLen, -segLen*2 - 4*segBuffer, 0},
	},
	"C": []r3.Vector{
		{segLen + 2*segBuffer, -segLen - 3*segBuffer, 0},
		{segLen + 2*segBuffer, -segLen*2 - 3*segBuffer, 0},
	},
	"B": []r3.Vector{
		{segLen + 2*segBuffer, -segBuffer, 0},
		{segLen + 2*segBuffer, -segBuffer - segLen, 0},
	},
}

var digitSegmentMap = map[string]string{
	".": "",
	"0": "ABCDEF",
	"1": "BC",
	"2": "ABGED",
	"3": "ABCDG",
	"4": "BCFG",
	"5": "ACDFG",
	"6": "BCDEFG",
	"7": "ABC",
	"8": "ABCDEFG",
	"9": "ABCDFG",
}
