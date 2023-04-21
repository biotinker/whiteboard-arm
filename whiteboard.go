package main

import (
	"context"
	"fmt"
	"github.com/golang/geo/r3"
	"strconv"
	"strings"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
	"go.viam.com/utils/rpc"

	servicepb "go.viam.com/api/service/motion/v1"
	"go.viam.com/rdk/services/motion"
)

// Constants used for calculating positions
const (
	cloudUrl = "" // Viam cloud URL
	secret   = "" // VIAM cloud secret

	zAdj = 3.5         // adjustment to glass contact altitude for both the marker and the eraser
	zBuf = zAdj + 50 // adjustment to non-writing holding height

	eraserBuf = zAdj - 1 // eraser held this much further away than marker

	armName = "xArm7" // name of Viam component to do the writing

	markerLen = 170. // Length of line from arm EE to marker tip
	eraserLen = 180. // Length of line from arm EE to eraser center 
	eraserX   = 90. // eraser X size in mm
	eraserY   = 45. // eraser Y size in mm

	// 7-segment line length and spacing
	segLen    = 100.
	segBuffer = 10.

	digitXoffset = 150. // width of a digit + the spacing to the next digit
	colonWidth   = 55. // Width to leave between hour and minute digits for a :
)

var (
	// Measured points on glass plane
	pt1 = r3.Vector{-430.35, -412, 699.97}
	pt2 = r3.Vector{-97.5, -404.28, 415.81}
	pt3 = r3.Vector{232.71, -400, 753.94}

	home7 = referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0, 0})

	v1    = pt2.Sub(pt1)
	v2    = pt3.Sub(pt1)
	pNorm = v1.Cross(v2).Normalize()

	allowWrite = []*servicepb.CollisionSpecification_AllowedFrameCollisions{
		// BUG workaround, will update when RDK is fixed
		&servicepb.CollisionSpecification_AllowedFrameCollisions{Frame1: "marker_origin", Frame2: "glass"},
	}
	allowErase = []*servicepb.CollisionSpecification_AllowedFrameCollisions{
		// BUG workaround, will update when RDK is fixed
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

// Generate the various transforms from the end of the xArm7 to the ends of the marker/eraser on the printed attachment
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

// Create the obstacles for things not to hit: arm pedestal, metal sides of the window, and of course the glass pane itself
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

func calibrate(motionService motion.Service, worldState *referenceframe.WorldState) {

	markerResource := resource.Name{Name: "marker"}
	orient := &spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 90}

	startPt := calibPoints[0]
	startPt.Z += zBuf
	goal := referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(startPt, orient))
	_, err := motionService.Move(context.Background(), markerResource, goal, worldState, nil, nil)
	if err != nil {
		fmt.Println(err)
	}
	for _, viamPt := range calibPoints {
		adjPt := viamPt
		adjPt.Z += zAdj
		goal = referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(adjPt, orient))
		_, err = motionService.Move(context.Background(), markerResource, goal, worldState, writeConstraint, nil)
	}
	endPt := calibPoints[len(calibPoints)-1]
	endPt.Z += zBuf
	goal = referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(endPt, orient))
	_, err = motionService.Move(context.Background(), markerResource, goal, worldState, writeConstraint, nil)
}

// Calling this will write the time on the defined plane, and keep it updated
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
				r3.Vector{2*digitXoffset + colonWidth + float64(1-i)*digitXoffset, 0, 0},
			)
		}
		for i, d := range minuteStr {
			changeDigit(
				motionService,
				worldState,
				digitSegmentMap[oldMinute[i]],
				digitSegmentMap[string(d)],
				r3.Vector{float64(1-i) * digitXoffset, 0, 0},
			)
		}
		oldHour = strings.Split(hourStr, "")
		oldMinute = strings.Split(minuteStr, "")
		time.Sleep(5 * time.Second)
	}
}

// This will determine which segment changes are necessary to update a single 7-segment digit, and will erase then draw the needed segs
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

// This will write one segment in a 7-segment display
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
	_, err = motionService.Move(context.Background(), markerResource, goal, worldState, linearConstraint, nil)
	if err != nil {
		fmt.Println(err)
	}
}

// This will erase one segment in a 7-segment display
func eraseSegment(motionService motion.Service, worldState *referenceframe.WorldState, segStr string, offset r3.Vector) {
	eraserResource := resource.Name{Name: "eraser"}
	eraserOrient := &spatialmath.OrientationVectorDegrees{OZ: -1}
	segPts := segments[segStr]

	var eraserPt1 r3.Vector
	var eraserPt2 r3.Vector

	// Determine start/end points from eraser and line dimensions
	if segPts[0].X == segPts[1].X {
		// vertical line
		eraserPt1 = segPts[0]
		eraserPt1.Y -= (eraserX/2) - 4
		eraserPt2 = segPts[1]
		eraserPt2.Y += (eraserX/2) - 4
		
		// Don't erase adjacent lines
		if segPts[0].X == 0 {
			// left side
			eraserPt1.X += eraserY/2 - segBuffer
			eraserPt2.X += eraserY/2 - segBuffer
		} else {
			// right side
			eraserPt1.X -= eraserY/2 - segBuffer
			eraserPt2.X -= eraserY/2 - segBuffer
		}
		
	} else {
		//horizontal line
		//~ eraserOrient = &spatialmath.OrientationVectorDegrees{OZ: -1}
		eraserPt1 = segPts[0]
		eraserPt1.X += (eraserY/2)
		eraserPt2 = segPts[1]
		eraserPt2.X -= (eraserY/2)
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
	// touch glass
	goal = referenceframe.NewPoseInFrame("glass", spatialmath.NewPose(eraserPt1.Add(offset), eraserOrient))
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

// calibration points; this just draws a big box
var calibPoints = []r3.Vector{
	{0, 0, 0},
	{600, 0, 0},
	{600, -300, 0},
	{0, -300, 0},
	{0, 0, 0},
}

// Define the line segments for each segment of a 7-segment display
//  _     A
// |_|  F   B
// |_|    G
//      E   C
//        D
var segments = map[string][]r3.Vector{
	"B": []r3.Vector{
		{0, -segBuffer, 0},
		{0, 0 - segBuffer - segLen, 0},
	},
	"C": []r3.Vector{
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
	"E": []r3.Vector{
		{segLen + 2*segBuffer, -segLen - 3*segBuffer, 0},
		{segLen + 2*segBuffer, -segLen*2 - 3*segBuffer, 0},
	},
	"F": []r3.Vector{
		{segLen + 2*segBuffer, -segBuffer, 0},
		{segLen + 2*segBuffer, -segBuffer - segLen, 0},
	},
}

// Specify which segments of a 7-segment display are used to draw each digit, plus "." to represent "empty"
var digitSegmentMap = map[string]string{
	".": "",
	"0": "ABCDEF",
	"1": "BC",
	"2": "ABGED",
	"3": "ABCDG",
	"4": "BCFG",
	"5": "ACDFG",
	"6": "ACDEFG",
	"7": "ABC",
	"8": "ABCDEFG",
	"9": "ABCDFG",
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
	//~ calibrate(motionService, worldState) // run this to simply draw a large box, helpful for checking plane calibration
	writeTime(motionService, worldState)
}
