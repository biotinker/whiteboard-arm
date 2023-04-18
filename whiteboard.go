package main

import (
	"github.com/golang/geo/r3"
	"fmt"
	"math"
	"context"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/referenceframe"
	//~ pb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils"
	"go.viam.com/utils/rpc"
	
	"go.viam.com/rdk/services/motion"
)


const (
	
	cloudUrl = ""
	secret = ""
	
	zBuf = 20
	
	startX = 200.
	startY = -400.
	startZ = 650.
	
	armName = "xArm7"
	
	markerLen = 170.
	eraserLen = 180.
)

var(
	// Measured points on glass plane
	pt1 = r3.Vector{-479.8, -412.3, 613.5}
	pt2 = r3.Vector{-62.4, -398.2, -147.4}
	pt3 = r3.Vector{216.1, -399.2, 734.4}
	
	home7 = referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0, 0})
	
	v1 = pt2.Sub(pt1)
	v2 = pt3.Sub(pt1)
	pNorm = v1.Cross(v2).Normalize()
)

// This will test solving the path to write the word "VIAM" on a whiteboard.
func GenerateTransforms() []*referenceframe.LinkInFrame {

	if pNorm.Y < 0 {
		pNorm = pNorm.Mul(-1)
	}

	glassFrame := referenceframe.NewLinkInFrame(
		"world",
		spatialmath.NewPose(pt3, &spatialmath.OrientationVectorDegrees{Theta: 90, OX: pNorm.X, OY:pNorm.Y, OZ:pNorm.Z}),
		"glass",
		nil,
	)

	markerOriginFrame := referenceframe.NewLinkInFrame(
		armName,
		spatialmath.NewPoseFromOrientation(&spatialmath.OrientationVectorDegrees{OY: -1, OZ: 1}),
		"marker_base",
		nil,
	)
	markerGeom, _ := spatialmath.NewCapsule(spatialmath.NewPoseFromPoint(r3.Vector{0, 0, markerLen/2}), 15, markerLen, "marker")
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
	eraserStemGeom, _ := spatialmath.NewCapsule(spatialmath.NewPoseFromPoint(r3.Vector{0, 0, eraserLen/2}), 15, markerLen, "eraser_stem")
	eraserStemFrame := referenceframe.NewLinkInFrame(
		"eraser_base",
		spatialmath.NewPoseFromPoint(r3.Vector{0, 0, eraserLen}),
		"eraser_stem",
		eraserStemGeom,
	)
	eraserGeom, _ := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{110,55,0}, "eraser")
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
	
	// Add the glass as a geometry
	glassDims := r3.Vector{X: 20000.0, Y: 20000.0, Z: 0.0}
	glassObj, _ := spatialmath.NewBox(spatialmath.NewZeroPose(), glassDims, "glass")
	obstacles = append(obstacles, glassObj)
	
	obstaclesInFrame = append(obstaclesInFrame, referenceframe.NewGeometriesInFrame(referenceframe.World, obstacles))
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
		Obstacles: obstacles,
		Transforms: transforms,
	}
	_ = worldState
	defer robot.Close(context.Background())
	logger.Info("Resources:")
	logger.Info(robot.ResourceNames())

	markerResource := resource.Name{Name:"marker"}
	
	markerPoseInGlass, err := motionService.GetPose(context.Background(), markerResource, "glass", transforms, nil)
	if err != nil {
		fmt.Println(err)
	}
	fmt.Println(spatialmath.PoseToProtobuf(markerPoseInGlass.Pose()))
	
	goal := referenceframe.NewPoseInFrame("glass", spatialmath.NewPoseFromPoint(r3.Vector{0, 0, zBuf}))
	_, err = motionService.Move(context.Background(), markerResource, goal, worldState, nil, nil)
	if err != nil {
		fmt.Println(err)
	}
}


// Write out the word "VIAM".
var viamPoints = [][]r3.Vector{
	[]r3.Vector{
		{0,0,0},
		{40, -100,0},
		{80, 0,0},
	},
	[]r3.Vector{
		{100,0,0},
		{100, -100,0},
	},
	[]r3.Vector{
		{120,-100,0},
		{160, 0,0},
		{200, -100,0},
	},
	[]r3.Vector{
		{140, -50,0},
		{180, -50,0},
	},
	[]r3.Vector{
		{220, -100,0},
		{250, 0,0},
		{280, -100,0},
		{310, 0,0},
		{340, -100,0},
	},
}
