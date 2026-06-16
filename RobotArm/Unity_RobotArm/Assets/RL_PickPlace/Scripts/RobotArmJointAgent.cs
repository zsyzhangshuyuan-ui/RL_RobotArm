using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;


public class RobotArmJointAgent : Agent
{
    //Break the task down into several smaller stages
    private enum TaskPhase
    {
        MoveToPick, //move to the pick target first
        PickPause,  //short wait while closing the gripper
        MoveToSafePoint,  //move to a safe point above the start and place areas
        MoveToReleaseArea,     //move to the release area above the place area
        DropPause  //short wait while opening the gripper
    }

  // Six joint transforms.
    [Header("Robot Joints")]
    public Transform jointA1;
    public Transform jointA2;
    public Transform jointA3;
    public Transform jointA4;
    public Transform jointA5;
    public Transform jointA6;

  // Local rotation axis for each joint
    [Header("Joint Axes")]
    public Vector3 axisA1 = Vector3.up;
    public Vector3 axisA2 = Vector3.forward;
    public Vector3 axisA3 = Vector3.forward;
    public Vector3 axisA4 = Vector3.right;
    public Vector3 axisA5 = Vector3.forward;
    public Vector3 axisA6 = Vector3.right;

    [Header("Joint Direction Multiplier")]
    public float dirA1 = 1f;
    public float dirA2 = 1f;
    public float dirA3 = 1f;
    public float dirA4 = 1f;
    public float dirA5 = 1f;
    public float dirA6 = 1f;

    // TCP, grasp point, object, and target points used by the task logic
    [Header("TCP / Grasp / Targets")]
    public Transform robotTCP;
    public Transform graspPoint;
    public Transform pickTarget;
    public Transform safePoint;
    public Transform targetPoint;
    public Transform placeArea;
    public Transform objectToPick;

   //Randomises the start and place areas
    [Header("Random Start / Place Area")]
    public bool randomizeStartAndPlace = true;
    public Transform robotStand;
    public Transform startArea;
    public float randomSquareSize = 2.0f;
    public float robotStandForbiddenSquareSize = 1.0f;
    public float minStartPlaceCenterDistance = 0.65f;
    public int randomSpawnMaxTries = 100;
    public Vector2 fallbackAreaHalfExtentsXZ = new Vector2(0.35f, 0.35f);
    public float startAreaSurfaceOffset = 0.025f;
    public float objectSurfaceClearance = 0.003f;
    public bool moveTargetPointWithPlaceArea = true;

  //// Dynamic safe point: After picking, the arm passes this point to reduce collisions and unsafe paths
    [Header("Dynamic Safe Point")]
    public bool useDynamicSafePoint = true;
    public float dynamicSafePointHeightOffset = 0.5f;
    public float safePointTieEpsilon = 0.001f;
    public float safePointApexAngle = 120f;

   //Automatically moves the pick target near the object top
    [Header("Auto Pick Target")]
    public bool autoUpdatePickTargetToObjectTop = true;
    public float pickTargetTopOffset = -0.015f;

  // Gripper controller
    [Header("Gripper Control")]
    public SimpleGripperController gripperController;

  //Joint movement speed and maximum steps per episode
    [Header("Movement")]
    public float jointSpeed = 15f;
    public int maxEpisodeSteps = 4000;

   // Reward and penalty settings for various task phases and conditions
    [Header("Pick Check")]
    public Vector3 pickTolerance = new Vector3(0.035f, 0.035f, 0.035f);
    public float pickDistanceRewardScale = 0.3f;

 // Pre-pick object push check to penalize pushing the object down too much. Too much pushing ends the episode
    [Header("Pre Pick Object Push Check")]
    public bool usePrePickPushCheck = true;
    public float prePickPushDownLimit = 0.035f;
    public float prePickPushPenalty = -3.0f;

 // After picking, the object bottom must stay above a safe carry height.
    [Header("Carry Height Rule")]
    public float minCarryObjectBottomHeight = 0.03f;
    public float belowCarryHeightPenalty = -6.0f;
    public float heightPriorityRewardScale = 0.3f;
    public float downwardAfterPickPenaltyScale = 8.0f;

  // Reward settings for the safe-point phase. Encourage the agent to move toward the safe point first
    [Header("Safe Point Priority")]
    public Vector3 safePointTolerance = new Vector3(0.15f, 0.12f, 0.15f);
    public float safePointArriveReward = 6.0f;
    public float safePointProgressRewardScale = 8.0f;
    public float safePointDistancePenaltyScale = 0.08f;
    public int safePointStuckStepLimit = 80;
    public float safePointStuckPenalty = -0.05f;

  //Timeout settings prevent the agent from wandering in one phase forever
    [Header("Phase Timeout / Anti Wandering")]
    public int safePointPhaseMaxSteps = 300;
    public float safePointTimeoutPenalty = -1.0f;
    public int releasePhaseMaxSteps = 500;
    public float releaseTimeoutPenalty = -1.5f;

  //Reduces backtracking during release. Getting closer is rewarded; moving away is penalised
    [Header("Release Anti Backtracking")]
    public float releaseBestProgressRewardScale = 5.0f;
    public float releaseBacktrackPenaltyScale = 1.0f;
    public float releaseBacktrackDeadZone = 0.03f;
    public float releaseNearDistance = 0.35f;
    public float releaseVeryNearDistance = 0.18f;
    public float releaseNearReward = 1.0f;
    public float releaseVeryNearReward = 2.0f;

  //Release height, arrival tolerance, and reward settings for moving to the target
    [Header("Target / Release Priority")]
    public float releaseHeight = 0.50f;
    public Vector3 releaseTolerance = new Vector3(0.07f, 0.07f, 0.07f);
    public float targetArriveReward = 4.0f;
    public float targetProgressRewardScale = 8.0f;
    public float targetDistancePenaltyScale = 0.04f;

  // Checks whether the held object tilts too much. Large tilt means failure
    [Header("Held Object Upright Check")]
    public bool enforceHeldObjectUpright = true;
    public float softHeldTiltAngle = 8f;
    public float maxHeldTiltAngle = 20f;
    public float heldTiltPenaltyScale = 0.08f;
    public float heldFlipPenalty = -8.0f;

  //Final success check: object must be inside the area, stable, not badly tilted, and not below the surface
    [Header("Final Place Area Check")]
    public Vector3 placeAreaHalfExtents = new Vector3(0.30f, 0.10f, 0.30f);
    public float stableVelocityThreshold = 0.12f;
    public float stableAngularVelocityThreshold = 0.8f;
    public float maxTiltAngle = 25f;

  // Reward and penalty settings for the final place check
    [Header("Distance Penalty")]
    public bool useDistancePenalty = true;
    public float pickDistancePenaltyScale = 0.02f;

  //If the agent makes no progress during picking, it receives a small penalty
    [Header("Pick Stuck Penalty")]
    public bool usePickStuckPenalty = true;
    public float minPickImprovement = 0.001f;
    public int pickStuckStepLimit = 80;
    public float pickStuckPenalty = -0.02f;
    public bool endEpisodeWhenPickStuckTooLong = false;
    public int pickStuckEndStepLimit = 180;
    public float pickStuckEndPenalty = -2.0f;

  //Hard limit to stop the object or important arm points from going through the PlaceArea surface
    [Header("Hard PlaceArea Surface Limit")]
    public bool usePlaceAreaSurfaceLimit = true;
    public float placeAreaSurfaceOffset = 0.025f;
    public float minHeightClearance = -0.005f;
    public float severeBelowSurfaceLimit = -0.03f;
    public float surfacePenalty = -0.05f;
    public float severeSurfacePenalty = -2.0f;
    public Transform[] armHeightCheckPoints;
 
   //Pause duration for picking and dropping, giving the gripper and physics time to react
    [Header("Pause Timing")]
    public float pickPauseDuration = 0.5f;
    public float dropWaitDuration = 1.0f;

  // Reward and penalty settings for various task phases and conditions
    [Header("Reward")]
    public float pickArriveReward = 1.0f;
    public float pickReward = 4.0f;
    public float finalPlaceSuccessReward = 8.0f;
    public float finalPlaceFailPenalty = -2.0f;
    public float timePenalty = -0.001f;
    public float movementRewardScale = 1.0f;

  //Action smoothing reduces sudden large joint movements
    [Header("Action Smoothing")]
    public bool smoothActions = true;
    public float actionSmoothFactor = 0.2f;

  // Drop settings: whether to enable gravity on the object after release
    [Header("Drop Settings")]
    public bool enableGravityAfterRelease = true;

    private Quaternion startA1;
    private Quaternion startA2;
    private Quaternion startA3;
    private Quaternion startA4;
    private Quaternion startA5;
    private Quaternion startA6;

    private Vector3 objectStartPosition;
    private Quaternion objectStartRotation;
    private Vector3 objectStartScale;
    private Transform objectStartParent;

    private TaskPhase phase = TaskPhase.MoveToPick;

    private bool isHolding = false;
    private int stepCount = 0;
    private int phaseStepCount = 0;

    private float previousDistance = 0f;
    private float phaseTimer = 0f;

    private int pickNoProgressSteps = 0;
    private int safePointNoProgressSteps = 0;

    private float previousObjectBottomHeight = 0f;
    private float prePickObjectStartBottomY = 0f;

    private float bestReleaseDistance = Mathf.Infinity;
    private bool releaseNearRewardGiven = false;
    private bool releaseVeryNearRewardGiven = false;

    private float[] smoothedActions = new float[6];
  
  // Initializes the agent, saving the starting joint rotations and object state
    public override void Initialize()
    {
        SaveStartJointRotations();

        if (objectToPick != null)
        {
            objectStartParent = objectToPick.parent;
            objectStartPosition = objectToPick.position;
            objectStartRotation = objectToPick.rotation;
            objectStartScale = objectToPick.localScale;
        }

        if (gripperController != null)
        {
            gripperController.OpenGripper();
        }
    }

  // Resets the agent and environment at the beginning of each episode, randomizing positions if enabled
    public override void OnEpisodeBegin()
    {
        ResetJoints();
     // Reset the object to its initial state
        if (randomizeStartAndPlace)
        {
            RandomizeStartPlaceAndObject();
        }
        else
        {
            ResetObject();
        }
      // After the object position changes, update the pick target to the object top
        if (autoUpdatePickTargetToObjectTop)
        {
            UpdatePickTargetToObjectTopCenter();
        }

        stepCount = 0;
        phaseStepCount = 0;
        phaseTimer = 0f;
        isHolding = false;
        phase = TaskPhase.MoveToPick;

        pickNoProgressSteps = 0;
        safePointNoProgressSteps = 0;

        bestReleaseDistance = Mathf.Infinity;
        releaseNearRewardGiven = false;
        releaseVeryNearRewardGiven = false;

        previousObjectBottomHeight = GetObjectBottomHeightAboveSurface();
        prePickObjectStartBottomY = GetObjectWorldBottomY();

        for (int i = 0; i < smoothedActions.Length; i++)
        {
            smoothedActions[i] = 0f;
        }

        if (gripperController != null)
        {
            gripperController.OpenGripper();
        }

        Transform activePoint = GetActiveEndPoint();

        if (activePoint != null && pickTarget != null)
        {
            previousDistance = Vector3.Distance(activePoint.position, pickTarget.position);
        }
    }

  // Sends the current state to the neural network. The policy uses these observations to choose actions
    public override void CollectObservations(VectorSensor sensor)
    {
        Transform activePoint = GetActiveEndPoint();

        Vector3 activePos = activePoint != null ? activePoint.position : Vector3.zero;
        Vector3 pickPos = pickTarget != null ? pickTarget.position : Vector3.zero;
        Vector3 currentTarget = GetCurrentTargetPoint();

      // First observe the six joint angles
        sensor.AddObservation(GetNormalizedJointAngle(jointA1, axisA1));
        sensor.AddObservation(GetNormalizedJointAngle(jointA2, axisA2));
        sensor.AddObservation(GetNormalizedJointAngle(jointA3, axisA3));
        sensor.AddObservation(GetNormalizedJointAngle(jointA4, axisA4));
        sensor.AddObservation(GetNormalizedJointAngle(jointA5, axisA5));
        sensor.AddObservation(GetNormalizedJointAngle(jointA6, axisA6));

      // Then observe the end point, pick target, and place area positions
        sensor.AddObservation(activePoint != null ? activePoint.localPosition : Vector3.zero);
        sensor.AddObservation(pickTarget != null ? pickTarget.localPosition : Vector3.zero);
        sensor.AddObservation(placeArea != null ? placeArea.localPosition : Vector3.zero);
      
      // Tell the network whether the object is currently held
        sensor.AddObservation(isHolding ? 1f : 0f);

        float surfaceY = GetPlaceAreaSurfaceY();

        float graspHeight = graspPoint != null ? graspPoint.position.y - surfaceY : 0f;
        float tcpHeight = robotTCP != null ? robotTCP.position.y - surfaceY : 0f;
        float objectBottomHeight = GetObjectBottomHeightAboveSurface();

        sensor.AddObservation(graspHeight);
        sensor.AddObservation(tcpHeight);
        sensor.AddObservation(objectBottomHeight);
     
     // Finally, observe the vectors from the active point to the pick target and current target
        Vector3 toPick = pickPos - activePos;
        sensor.AddObservation(toPick);

        Vector3 toCurrentTarget = currentTarget - activePos;
        sensor.AddObservation(toCurrentTarget);
    }

  // Receives actions from the agent and applies them to the robot joints, while also handling task phase logic and reward calculations
    public override void OnActionReceived(ActionBuffers actions)
    {
        stepCount++;
        phaseStepCount++;

        if (phase == TaskPhase.PickPause)
        {
            HandlePickPause();
            CheckMaxSteps();
            return;
        }

        if (phase == TaskPhase.DropPause)
        {
            HandleDropPause();
            CheckMaxSteps();
            return;
        }
      
      // Apply actions first, then check the result of the movement
        ApplyJointActions(actions);

        if (!isHolding && phase == TaskPhase.MoveToPick)
        {
            bool failedByPushingObject = ApplyPrePickPushCheck();

            if (failedByPushingObject)
            {
                return;
            }
        }

        if (usePlaceAreaSurfaceLimit)
        {
            bool checkObject = isHolding && phase == TaskPhase.MoveToReleaseArea;
            bool failedBySurface = ApplyPlaceAreaSurfaceLimit(checkObject);

            if (failedBySurface)
            {
                return;
            }
        }

        if (isHolding)
        {
            bool failedByCarry = ApplyCarryHeightRule();

            if (failedByCarry)
            {
                return;
            }

            if (enforceHeldObjectUpright)
            {
                bool failedByTilt = ApplyHeldObjectUprightCheck();

                if (failedByTilt)
                {
                    return;
                }
            }

            ApplySecondaryHeightReward();
        }

        if (phase == TaskPhase.MoveToPick)
        {
            MoveToPickTarget();
        }
        else if (phase == TaskPhase.MoveToSafePoint)
        {
            MoveToSafePoint();
        }
        else if (phase == TaskPhase.MoveToReleaseArea)
        {
            MoveToReleaseArea();
        }

        CheckMaxSteps();
    }
  
  //Converts the six continuous action values into rotations for the six joints
    private void ApplyJointActions(ActionBuffers actions)
    {
        float delta = jointSpeed * Time.deltaTime;

        for (int i = 0; i < 6; i++)
        {
            float rawAction = Mathf.Clamp(actions.ContinuousActions[i], -1f, 1f);

            if (smoothActions)
            {
                smoothedActions[i] = Mathf.Lerp(smoothedActions[i], rawAction, actionSmoothFactor);
            }
            else
            {
                smoothedActions[i] = rawAction;
            }
        }

        RotateJoint(jointA1, axisA1, smoothedActions[0] * dirA1 * delta);
        RotateJoint(jointA2, axisA2, smoothedActions[1] * dirA2 * delta);
        RotateJoint(jointA3, axisA3, smoothedActions[2] * dirA3 * delta);
        RotateJoint(jointA4, axisA4, smoothedActions[3] * dirA4 * delta);
        RotateJoint(jointA5, axisA5, smoothedActions[4] * dirA5 * delta);
        RotateJoint(jointA6, axisA6, smoothedActions[5] * dirA6 * delta);
    }

 //-------------------------------------------------------------------------------------------------------------------
  //Phase 1: move the grasp point toward the pick target. Getting closer is rewarded; arrival closes the gripper
    private void MoveToPickTarget()
    {
        Transform activePoint = GetActiveEndPoint();

        if (activePoint == null || pickTarget == null)
        {
            AddReward(-1.0f);
            EndEpisode();
            return;
        }

        float currentDistance = Vector3.Distance(activePoint.position, pickTarget.position);

        AddReward(timePenalty);

        float improvement = previousDistance - currentDistance;
        AddReward(improvement * movementRewardScale);

      // Reward for getting closer to the pick target, scaled by distance and time
        AddReward(1.0f / (1.0f + currentDistance) * pickDistanceRewardScale * Time.deltaTime);

        if (useDistancePenalty)
        {
            AddReward(-currentDistance * pickDistancePenaltyScale * Time.deltaTime);
        }

        if (usePickStuckPenalty)
        {
            if (improvement > minPickImprovement)
            {
                pickNoProgressSteps = 0;
            }
            else
            {
                pickNoProgressSteps++;
            }

            if (pickNoProgressSteps > pickStuckStepLimit)
            {
                AddReward(pickStuckPenalty);
            }

            if (endEpisodeWhenPickStuckTooLong && pickNoProgressSteps > pickStuckEndStepLimit)
            {
                AddReward(pickStuckEndPenalty);
                EndEpisode();
                return;
            }
        }

        previousDistance = currentDistance;

        Vector3 error = activePoint.position - pickTarget.position;

        bool xAligned = Mathf.Abs(error.x) < pickTolerance.x;
        bool yAligned = Mathf.Abs(error.y) < pickTolerance.y;
        bool zAligned = Mathf.Abs(error.z) < pickTolerance.z;

        if (xAligned && yAligned && zAligned)
        {
            //Phase 1.1: pause and close the gripper.
            phase = TaskPhase.PickPause;
            phaseStepCount = 0;
            phaseTimer = 0f;
            pickNoProgressSteps = 0;

            if (gripperController != null)
            {
                gripperController.CloseGripper();
            }

            AddReward(pickArriveReward);
        }
    }

   // Pre-pick push check: if the object is pushed down too much before picking, penalize and end the episode
    private bool ApplyPrePickPushCheck()
    {
        if (!usePrePickPushCheck)
            return false;

        if (objectToPick == null)
            return false;

        float currentBottomY = GetObjectWorldBottomY();

        if (currentBottomY < prePickObjectStartBottomY - prePickPushDownLimit)
        {
            AddReward(prePickPushPenalty);
            EndEpisode();
            return true;
        }

        return false;
    }

   // Phase 2: pause after reaching the pick target, allowing the gripper to close and the object to be picked up
    private void HandlePickPause()
    {
        AddReward(timePenalty);

        phaseTimer += Time.deltaTime;

        if (phaseTimer >= pickPauseDuration)
        {
           //Phase 2.1: after the pause, check if the object is held and move to the safe point phase
            PickObject();
            isHolding = true;

            AddReward(pickReward);

            previousObjectBottomHeight = GetObjectBottomHeightAboveSurface();
          
          // Update the safe point position dynamically based on the start and place areas
            UpdateDynamicSafePoint();

            Transform activePoint = GetActiveEndPoint();

            if (safePoint != null && activePoint != null)
            {
                previousDistance = Vector3.Distance(activePoint.position, safePoint.position);
                safePointNoProgressSteps = 0;
                phaseStepCount = 0;
                phase = TaskPhase.MoveToSafePoint;
            }
            else
            {
                StartReleasePhase(activePoint);
            }

            phaseTimer = 0f;
        }
    }

  // Phase 3: move the grasp point toward the safe point. Getting closer is rewarded; arrival moves to the release area phase
    private void MoveToSafePoint()
    {
        Transform activePoint = GetActiveEndPoint();

        if (activePoint == null || safePoint == null)
        {
            AddReward(-1.0f);
            EndEpisode();
            return;
        }

        float currentDistance = Vector3.Distance(activePoint.position, safePoint.position);

        AddReward(timePenalty);

        float improvement = previousDistance - currentDistance;

        AddReward(improvement * safePointProgressRewardScale);

        if (useDistancePenalty)
        {
            AddReward(-currentDistance * safePointDistancePenaltyScale * Time.deltaTime);
        }

        AddReward(1.0f / (1.0f + currentDistance) * 0.6f * Time.deltaTime);

        if (improvement > minPickImprovement)
        {
            safePointNoProgressSteps = 0;
        }
        else
        {
            safePointNoProgressSteps++;
        }

        if (safePointNoProgressSteps > safePointStuckStepLimit)
        {
            AddReward(safePointStuckPenalty);
        }

        previousDistance = currentDistance;

        Vector3 error = activePoint.position - safePoint.position;

        bool xAligned = Mathf.Abs(error.x) < safePointTolerance.x;
        bool yAligned = Mathf.Abs(error.y) < safePointTolerance.y;
        bool zAligned = Mathf.Abs(error.z) < safePointTolerance.z;

        if (xAligned && yAligned && zAligned)
        {
            //Phase 3.1: give a larger one-time reward after reaching the safe point, then enter the release phase
            AddReward(safePointArriveReward);

            StartReleasePhase(activePoint);

            phaseTimer = 0f;
            safePointNoProgressSteps = 0;
            return;
        }

        if (phaseStepCount > safePointPhaseMaxSteps)
        {
            AddReward(safePointTimeoutPenalty);

            StartReleasePhase(activePoint);

            phaseTimer = 0f;
            safePointNoProgressSteps = 0;
            return;
        }
    }
  
  // Phase 4: release phase. Before entering the release phase, reset distance tracking and one-time reward flags
    private void StartReleasePhase(Transform activePoint)
    {
        if (activePoint != null)
        {
            bestReleaseDistance = Vector3.Distance(activePoint.position, GetReleasePoint());
            previousDistance = bestReleaseDistance;
        }
        else
        {
            bestReleaseDistance = Mathf.Infinity;
        }

        releaseNearRewardGiven = false;
        releaseVeryNearRewardGiven = false;

        phaseStepCount = 0;
        phase = TaskPhase.MoveToReleaseArea;
    }

 // Phase 4: move to the release point. Moving closer is rewarded; backtracking is penalised
    private void MoveToReleaseArea()
    {
        Transform activePoint = GetActiveEndPoint();

        if (activePoint == null)
        {
            AddReward(-1.0f);
            EndEpisode();
            return;
        }

        Vector3 releasePoint = GetReleasePoint();

        float currentDistance = Vector3.Distance(activePoint.position, releasePoint);

        AddReward(timePenalty);

        float improvement = previousDistance - currentDistance;

        AddReward(improvement * targetProgressRewardScale);

        if (useDistancePenalty)
        {
            AddReward(-currentDistance * targetDistancePenaltyScale * Time.deltaTime);
        }

      //Track the best distance to the release point so backtracking can be penalised
        AddReward(1.0f / (1.0f + currentDistance) * 0.6f * Time.deltaTime);

        if (currentDistance < bestReleaseDistance)
        {
            float bestImprovement = bestReleaseDistance - currentDistance;
            AddReward(bestImprovement * releaseBestProgressRewardScale);
            bestReleaseDistance = currentDistance;
        }
        else
        {
            float backtrackDistance = currentDistance - bestReleaseDistance;

            if (backtrackDistance > releaseBacktrackDeadZone)
            {
                AddReward(-backtrackDistance * releaseBacktrackPenaltyScale * Time.deltaTime);
            }
        }

        if (!releaseNearRewardGiven && currentDistance < releaseNearDistance)
        {
            AddReward(releaseNearReward);
            releaseNearRewardGiven = true;
        }

        if (!releaseVeryNearRewardGiven && currentDistance < releaseVeryNearDistance)
        {
            AddReward(releaseVeryNearReward);
            releaseVeryNearRewardGiven = true;
        }

        if (phaseStepCount > releasePhaseMaxSteps)
        {
            AddReward(releaseTimeoutPenalty);
            EndEpisode();
            return;
        }

        previousDistance = currentDistance;

        Vector3 error = activePoint.position - releasePoint;

        bool xAligned = Mathf.Abs(error.x) < releaseTolerance.x;
        bool yAligned = Mathf.Abs(error.y) < releaseTolerance.y;
        bool zAligned = Mathf.Abs(error.z) < releaseTolerance.z;

        if (xAligned && yAligned && zAligned)
        {  //Phase 4.1: pause and open the gripper to release the object and move to the final wait phase
            ReleaseObject();

            AddReward(targetArriveReward);

            phase = TaskPhase.DropPause;
            phaseStepCount = 0;
            phaseTimer = 0f;
        }
    }

   //---------------------------------------------------------------------------------------------------------------
   // Randomly places the start/place areas and spawns the object on the start area surface
    private void RandomizeStartPlaceAndObject()
    {
        if (robotStand == null || startArea == null || placeArea == null || objectToPick == null)
        {
            ResetObject();
            return;
        }

        Vector3 center = GetRobotStandBottomCenter();

        float startY = startArea.position.y;
        float placeY = placeArea.position.y;

        bool foundValidPosition = false;
      
      //Try several random layouts until a valid one is found
        for (int i = 0; i < randomSpawnMaxTries; i++)
        {
            startArea.position = GetRandomPointInSquare(center, startY);
            placeArea.position = GetRandomPointInSquare(center, placeY);

            Physics.SyncTransforms();

            if (IsValidStartPlaceLayout(center))
            {
                foundValidPosition = true;
                break;
            }
        }

        if (!foundValidPosition)
        {
            ForceGenerateFallbackLayout(center, startY, placeY);
        }

        if (moveTargetPointWithPlaceArea && targetPoint != null)
        {
            targetPoint.position = placeArea.position + Vector3.up * releaseHeight;
        }

        ResetObjectPhysicsForRandomSpawn();

        float startSurfaceY = GetAreaSurfaceY(startArea, startAreaSurfaceOffset);

        objectToPick.position = new Vector3(
            startArea.position.x,
            startSurfaceY + 0.2f,
            startArea.position.z
        );

        objectToPick.rotation = objectStartRotation;
        objectToPick.localScale = objectStartScale;

        Physics.SyncTransforms();

        Bounds objectBounds;

        if (TryGetObjectBounds(out objectBounds))
        {
            float desiredBottomY = startSurfaceY + objectSurfaceClearance;
            float moveUp = desiredBottomY - objectBounds.min.y;
            objectToPick.position += Vector3.up * moveUp;
        }

        Physics.SyncTransforms();
    }
  
  //Checks whether the random layout is valid: not too close, not overlapping, and not inside the forbidden stand area
    private bool IsValidStartPlaceLayout(Vector3 robotCenter)
    {
        bool centerTooClose = IsStartPlaceCenterTooClose();
        bool areaOverlapping = AreStartAndPlaceAreasOverlappingXZ();

        bool startInsideForbiddenArea =
            IsInsideRobotStandForbiddenSquare(startArea.position, robotCenter);

        bool placeInsideForbiddenArea =
            IsInsideRobotStandForbiddenSquare(placeArea.position, robotCenter);

        return
            !centerTooClose &&
            !areaOverlapping &&
            !startInsideForbiddenArea &&
            !placeInsideForbiddenArea;
    }

   //Fallback layout
    private void ForceGenerateFallbackLayout(Vector3 center, float startY, float placeY)
    {
        Vector3[] candidateDirections =
        {
            new Vector3(1f, 0f, 0f),
            new Vector3(-1f, 0f, 0f),
            new Vector3(0f, 0f, 1f),
            new Vector3(0f, 0f, -1f),
            new Vector3(1f, 0f, 1f).normalized,
            new Vector3(-1f, 0f, 1f).normalized,
            new Vector3(1f, 0f, -1f).normalized,
            new Vector3(-1f, 0f, -1f).normalized
        };

        float halfSize = randomSquareSize * 0.5f;
        float forbiddenHalfSize = robotStandForbiddenSquareSize * 0.5f;

        float safeRadius = Mathf.Max(forbiddenHalfSize + 0.25f, minStartPlaceCenterDistance);

        for (int i = 0; i < candidateDirections.Length; i++)
        {
            Vector3 dir = candidateDirections[i];

            Vector3 startPos = center + dir * safeRadius;
            Vector3 placePos = center - dir * safeRadius;

            startArea.position = ClampPointToRandomSquare(startPos, center, startY);
            placeArea.position = ClampPointToRandomSquare(placePos, center, placeY);

            Physics.SyncTransforms();

            if (IsValidStartPlaceLayout(center))
            {
                return;
            }
        }

        startArea.position = new Vector3(
            center.x + halfSize * 0.8f,
            startY,
            center.z + halfSize * 0.8f
        );

        placeArea.position = new Vector3(
            center.x - halfSize * 0.8f,
            placeY,
            center.z - halfSize * 0.8f
        );

        Physics.SyncTransforms();
    }
  
  //Checks whether inside the forbidden square around the robot stand
    private bool IsInsideRobotStandForbiddenSquare(Vector3 position, Vector3 robotCenter)
    {
        float halfForbiddenSize = robotStandForbiddenSquareSize * 0.5f;

        bool insideX = Mathf.Abs(position.x - robotCenter.x) < halfForbiddenSize;
        bool insideZ = Mathf.Abs(position.z - robotCenter.z) < halfForbiddenSize;

        return insideX && insideZ;
    }
   
   // Dynamically calculates the safe point from the start/place areas. 
   // It uses a triangle apex and chooses the side farther from the robot stand
    private void UpdateDynamicSafePoint()
    {
        if (!useDynamicSafePoint)
            return;

        if (safePoint == null || startArea == null || placeArea == null)
            return;

        Vector3 start = startArea.position;
        Vector3 place = placeArea.position;
        Vector3 robotCenter = robotStand != null ? GetRobotStandBottomCenter() : Vector3.zero;

        Vector3 startXZ = new Vector3(start.x, 0f, start.z);
        Vector3 placeXZ = new Vector3(place.x, 0f, place.z);
        Vector3 robotCenterXZ = new Vector3(robotCenter.x, 0f, robotCenter.z);

        Vector3 baseVector = placeXZ - startXZ;
        float baseLength = baseVector.magnitude;

        Vector3 midXZ = (startXZ + placeXZ) * 0.5f;

        if (baseLength < 0.001f)
        {
            safePoint.position = new Vector3(
                midXZ.x,
                Mathf.Max(start.y, place.y) + dynamicSafePointHeightOffset,
                midXZ.z
            );
            return;
        }

        Vector3 baseDir = baseVector.normalized;

        Vector3 rightPerpendicular = new Vector3(
            baseDir.z,
            0f,
            -baseDir.x
        );

        Vector3 leftPerpendicular = -rightPerpendicular;

        float angleRad = safePointApexAngle * Mathf.Deg2Rad;

        float halfBase = baseLength * 0.5f;
        float apexHeight = halfBase / Mathf.Tan(angleRad * 0.5f);

        Vector3 rightCandidateXZ = midXZ + rightPerpendicular * apexHeight;
        Vector3 leftCandidateXZ = midXZ + leftPerpendicular * apexHeight;

        float rightDistanceToRobot = Vector3.Distance(rightCandidateXZ, robotCenterXZ);
        float leftDistanceToRobot = Vector3.Distance(leftCandidateXZ, robotCenterXZ);

        Vector3 chosenXZ;
      
      //Choose the side farther from the robot stand to avoid the risky area near the base
        if (rightDistanceToRobot >= leftDistanceToRobot - safePointTieEpsilon)
        {
            chosenXZ = rightCandidateXZ;
        }
        else
        {
            chosenXZ = leftCandidateXZ;
        }

        float baseY = Mathf.Max(startArea.position.y, placeArea.position.y);

        safePoint.position = new Vector3(
            chosenXZ.x,
            baseY + dynamicSafePointHeightOffset,
            chosenXZ.z
        );
    }
   
  //If the start and place centers are too close, the layout is considered invalid 
    private bool IsStartPlaceCenterTooClose()
    {
        if (startArea == null || placeArea == null)
            return true;

        Vector2 startXZ = new Vector2(startArea.position.x, startArea.position.z);
        Vector2 placeXZ = new Vector2(placeArea.position.x, placeArea.position.z);

        return Vector2.Distance(startXZ, placeXZ) < minStartPlaceCenterDistance;
    }

   //Checks whether the two areas overlap
    private bool AreStartAndPlaceAreasOverlappingXZ()
    {
        Bounds startBounds;
        Bounds placeBounds;

        bool hasStartBounds = TryGetAreaBounds(startArea, out startBounds);
        bool hasPlaceBounds = TryGetAreaBounds(placeArea, out placeBounds);

        if (!hasStartBounds || !hasPlaceBounds)
        {
            return IsFallbackAreaOverlappingXZ();
        }

        bool overlapX = startBounds.min.x <= placeBounds.max.x && startBounds.max.x >= placeBounds.min.x;
        bool overlapZ = startBounds.min.z <= placeBounds.max.z && startBounds.max.z >= placeBounds.min.z;

        return overlapX && overlapZ;
    }

    private bool IsFallbackAreaOverlappingXZ()
    {
        if (startArea == null || placeArea == null)
            return true;

        float startMinX = startArea.position.x - fallbackAreaHalfExtentsXZ.x;
        float startMaxX = startArea.position.x + fallbackAreaHalfExtentsXZ.x;
        float startMinZ = startArea.position.z - fallbackAreaHalfExtentsXZ.y;
        float startMaxZ = startArea.position.z + fallbackAreaHalfExtentsXZ.y;

        float placeMinX = placeArea.position.x - fallbackAreaHalfExtentsXZ.x;
        float placeMaxX = placeArea.position.x + fallbackAreaHalfExtentsXZ.x;
        float placeMinZ = placeArea.position.z - fallbackAreaHalfExtentsXZ.y;
        float placeMaxZ = placeArea.position.z + fallbackAreaHalfExtentsXZ.y;

        bool overlapX = startMinX <= placeMaxX && startMaxX >= placeMinX;
        bool overlapZ = startMinZ <= placeMaxZ && startMaxZ >= placeMinZ;

        return overlapX && overlapZ;
    }

   //Tries to calculate the bounds of an area based on its colliders or renderers. If no valid bounds are found, returns false
    private bool TryGetAreaBounds(Transform area, out Bounds bounds)
    {
        bounds = new Bounds(Vector3.zero, Vector3.zero);

        if (area == null)
            return false;

        Collider[] colliders = area.GetComponentsInChildren<Collider>();

        bool hasBounds = false;

        for (int i = 0; i < colliders.Length; i++)
        {
            Collider col = colliders[i];

            if (col == null || !col.enabled)
                continue;

            if (!hasBounds)
            {
                bounds = col.bounds;
                hasBounds = true;
            }
            else
            {
                bounds.Encapsulate(col.bounds);
            }
        }

        if (hasBounds)
            return true;

        Renderer[] renderers = area.GetComponentsInChildren<Renderer>();

        for (int i = 0; i < renderers.Length; i++)
        {
            Renderer r = renderers[i];

            if (r == null || !r.enabled)
                continue;

            if (!hasBounds)
            {
                bounds = r.bounds;
                hasBounds = true;
            }
            else
            {
                bounds.Encapsulate(r.bounds);
            }
        }

        return hasBounds;
    }

    private Vector3 GetRandomPointInSquare(Vector3 center, float y)
    {
        float halfSize = randomSquareSize * 0.5f;

        float x = Random.Range(center.x - halfSize, center.x + halfSize);
        float z = Random.Range(center.z - halfSize, center.z + halfSize);

        return new Vector3(x, y, z);
    }

    private Vector3 ClampPointToRandomSquare(Vector3 point, Vector3 center, float y)
    {
        float halfSize = randomSquareSize * 0.5f;

        float x = Mathf.Clamp(point.x, center.x - halfSize, center.x + halfSize);
        float z = Mathf.Clamp(point.z, center.z - halfSize, center.z + halfSize);

        return new Vector3(x, y, z);
    }

  //Finds the bottom center of the robot stand, used as the centre of randomisation and forbidden area
    private Vector3 GetRobotStandBottomCenter()
    {
        if (robotStand == null)
            return Vector3.zero;

        Bounds bounds = new Bounds(Vector3.zero, Vector3.zero);
        bool hasBounds = false;

        Collider[] colliders = robotStand.GetComponentsInChildren<Collider>();

        for (int i = 0; i < colliders.Length; i++)
        {
            Collider col = colliders[i];

            if (col == null || !col.enabled)
                continue;

            if (!hasBounds)
            {
                bounds = col.bounds;
                hasBounds = true;
            }
            else
            {
                bounds.Encapsulate(col.bounds);
            }
        }

        if (hasBounds)
        {
            return new Vector3(bounds.center.x, bounds.min.y, bounds.center.z);
        }

        Renderer[] renderers = robotStand.GetComponentsInChildren<Renderer>();

        for (int i = 0; i < renderers.Length; i++)
        {
            Renderer r = renderers[i];

            if (r == null || !r.enabled)
                continue;

            if (!hasBounds)
            {
                bounds = r.bounds;
                hasBounds = true;
            }
            else
            {
                bounds.Encapsulate(r.bounds);
            }
        }

        if (hasBounds)
        {
            return new Vector3(bounds.center.x, bounds.min.y, bounds.center.z);
        }

        return robotStand.position;
    }

   // Resets the object's physics properties and transforms to prepare for a random spawn
    private void ResetObjectPhysicsForRandomSpawn()
    {
        if (objectToPick == null)
            return;

        objectToPick.gameObject.SetActive(true);
        objectToPick.SetParent(objectStartParent, true);
        objectToPick.rotation = objectStartRotation;
        objectToPick.localScale = objectStartScale;

        Rigidbody rb = objectToPick.GetComponent<Rigidbody>();

        if (rb != null)
        {
            rb.isKinematic = false;
            rb.useGravity = true;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }

    private float GetAreaSurfaceY(Transform area, float fallbackOffset)
    {
        if (area == null)
            return 0f;

        Collider[] colliders = area.GetComponentsInChildren<Collider>();

        for (int i = 0; i < colliders.Length; i++)
        {
            Collider col = colliders[i];

            if (col == null || !col.enabled)
                continue;

            if (col.isTrigger)
                continue;

            return col.bounds.max.y;
        }

        return area.position.y + fallbackOffset;
    }
  
  //---------------------------------------------------------------------------------------------------------------
  //Checks carry height. Too low gives a penalty; seriously too low ends the episode
    private bool ApplyCarryHeightRule()
    {
        float objectBottomHeight = GetObjectBottomHeightAboveSurface();

        if (objectBottomHeight < minCarryObjectBottomHeight)
        {
            AddReward(belowCarryHeightPenalty * Time.deltaTime);

            if (objectBottomHeight < minCarryObjectBottomHeight - 0.06f)
            {
                AddReward(belowCarryHeightPenalty);
                EndEpisode();
                return true;
            }
        }

        return false;
    }
  
  //After picking, gives a small reward for lifting higher and penalises dropping lower
    private void ApplySecondaryHeightReward()
    {
        float currentObjectBottomHeight = GetObjectBottomHeightAboveSurface();
        float heightImprovement = currentObjectBottomHeight - previousObjectBottomHeight;

        if (heightImprovement > 0f)
        {
            AddReward(heightImprovement * heightPriorityRewardScale);
        }
        else
        {
            AddReward(heightImprovement * downwardAfterPickPenaltyScale);
        }

        previousObjectBottomHeight = currentObjectBottomHeight;
    }

   //Checks whether the held object stays roughly upright. Small tilt is penalised; large tilt fails the episode
    private bool ApplyHeldObjectUprightCheck()
    {
        if (objectToPick == null)
            return false;

        float tiltAngle = Vector3.Angle(objectToPick.up, Vector3.up);

        if (tiltAngle > softHeldTiltAngle)
        {
            float extraTilt = tiltAngle - softHeldTiltAngle;
            AddReward(-extraTilt * heldTiltPenaltyScale * Time.deltaTime);
        }

        if (tiltAngle > maxHeldTiltAngle)
        {
            AddReward(heldFlipPenalty);
            EndEpisode();
            return true;
        }

        return false;
    }
 
 //Waits after dropping, then checks final success after the object has time to settle
    private void HandleDropPause()
    {
        phaseTimer += Time.deltaTime;

        if (phaseTimer >= dropWaitDuration)
        {
            bool success = CheckFinalPlacementSuccess();

            if (success)
            {
                AddReward(finalPlaceSuccessReward);
            }
            else
            {
                AddReward(finalPlaceFailPenalty);
            }

            EndEpisode();
        }
    }

    private Vector3 GetCurrentTargetPoint()
    {
        if (phase == TaskPhase.MoveToPick || phase == TaskPhase.PickPause)
        {
            if (pickTarget != null)
                return pickTarget.position;
        }

        if (phase == TaskPhase.MoveToSafePoint)
        {
            if (safePoint != null)
                return safePoint.position;
        }

        if (phase == TaskPhase.MoveToReleaseArea || phase == TaskPhase.DropPause)
        {
            return GetReleasePoint();
        }

        return Vector3.zero;
    }

    private Vector3 GetReleasePoint()
    {
        if (targetPoint != null)
            return targetPoint.position;

        if (placeArea == null)
            return Vector3.zero;

        return placeArea.position + Vector3.up * releaseHeight;
    }

  //Final success check: inside the area, stable, not too tilted, and not below the surface
    private bool CheckFinalPlacementSuccess()
    {
        if (objectToPick == null || placeArea == null)
            return false;

        Bounds objectBounds;

        if (!TryGetObjectBounds(out objectBounds))
            return false;

        Vector3 objectCenter = objectBounds.center;
        Vector3 localToArea = placeArea.InverseTransformPoint(objectCenter);

        bool insideX = Mathf.Abs(localToArea.x) <= placeAreaHalfExtents.x;
        bool insideZ = Mathf.Abs(localToArea.z) <= placeAreaHalfExtents.z;

        bool insideArea = insideX && insideZ;

        Rigidbody rb = objectToPick.GetComponent<Rigidbody>();

        bool stable = true;

        if (rb != null)
        {
            stable =
                rb.linearVelocity.magnitude < stableVelocityThreshold &&
                rb.angularVelocity.magnitude < stableAngularVelocityThreshold;
        }

        float tilt = Vector3.Angle(objectToPick.up, Vector3.up);
        bool notTooTilted = tilt < maxTiltAngle;

        float surfaceY = GetPlaceAreaSurfaceY();
        bool notBelowSurface = objectBounds.min.y >= surfaceY - minHeightClearance;

       //All 4 conditions must be true for a successful
        return insideArea && stable && notTooTilted && notBelowSurface;
    }
   
   //Checks whether the object or key arm points go below the PlaceArea surface, preventing clipping
    private bool ApplyPlaceAreaSurfaceLimit(bool checkObject)
    {
        if (placeArea == null)
            return false;

        float surfaceY = GetPlaceAreaSurfaceY();
        bool failed = false;

        if (checkObject && objectToPick != null)
        {
            Bounds objectBounds;

            if (TryGetObjectBounds(out objectBounds))
            {
                float objectBottomY = objectBounds.min.y;

                if (objectBottomY < surfaceY + minHeightClearance)
                {
                    AddReward(surfacePenalty);

                    if (objectBottomY < surfaceY + severeBelowSurfaceLimit)
                    {
                        failed = true;
                    }
                }
            }
        }

        Transform[] points = GetArmHeightCheckPoints();

        for (int i = 0; i < points.Length; i++)
        {
            Transform p = points[i];

            if (p == null)
                continue;

            if (p.position.y < surfaceY + minHeightClearance)
            {
                AddReward(surfacePenalty);

                if (p.position.y < surfaceY + severeBelowSurfaceLimit)
                {
                    failed = true;
                }
            }
        }

        if (failed)
        {
            AddReward(severeSurfacePenalty);
            EndEpisode();
            return true;
        }

        return false;
    }

  // Moves the pick target near the top center of the object
    private void UpdatePickTargetToObjectTopCenter()
    {
        if (pickTarget == null || objectToPick == null)
            return;

        Bounds bounds;

        if (!TryGetObjectBounds(out bounds))
            return;

        Vector3 topCenter = new Vector3(
            bounds.center.x,
            bounds.max.y + pickTargetTopOffset,
            bounds.center.z
        );

        pickTarget.position = topCenter;
    }
  
  //Gets the surface height of PlaceArea
    private float GetPlaceAreaSurfaceY()
    {
        if (placeArea == null)
            return 0f;

        Collider[] colliders = placeArea.GetComponentsInChildren<Collider>();

        for (int i = 0; i < colliders.Length; i++)
        {
            Collider col = colliders[i];

            if (col == null || !col.enabled)
                continue;

            if (col.isTrigger)
                continue;

            return col.bounds.max.y;
        }

        return placeArea.position.y + placeAreaSurfaceOffset;
    }

  //Calculates object bottom height relative to the PlaceArea surface
    private float GetObjectBottomHeightAboveSurface()
    {
        if (objectToPick == null || placeArea == null)
            return 0f;

        Bounds bounds;

        if (!TryGetObjectBounds(out bounds))
            return 0f;

        return bounds.min.y - GetPlaceAreaSurfaceY();
    }

    private float GetObjectWorldBottomY()
    {
        if (objectToPick == null)
            return 0f;

        Bounds bounds;

        if (TryGetObjectBounds(out bounds))
        {
            return bounds.min.y;
        }

        return objectToPick.position.y;
    }

    private Transform[] GetArmHeightCheckPoints()
    {
        if (armHeightCheckPoints != null && armHeightCheckPoints.Length > 0)
            return armHeightCheckPoints;

        return new Transform[]
        {
            robotTCP,
            graspPoint
        };
    }
 
  //Picks the object by making it kinematic and parenting it to the grasp point
    private void PickObject()
    {
        if (objectToPick == null)
            return;

        Transform attachPoint = GetActiveEndPoint();

        if (attachPoint == null)
            return;

        Rigidbody rb = objectToPick.GetComponent<Rigidbody>();

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.useGravity = false;
            rb.isKinematic = true;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
        }

        objectToPick.SetParent(attachPoint, true);
    }
  
  //Releases the object by unparenting it and making it non-kinematic so physics can take over
    private void ReleaseObject()
    {
        if (objectToPick == null)
            return;

        if (gripperController != null)
        {
            gripperController.OpenGripper();
        }

        objectToPick.SetParent(objectStartParent, true);

        Rigidbody rb = objectToPick.GetComponent<Rigidbody>();

        if (rb != null)
        {
            rb.isKinematic = false;
            rb.useGravity = enableGravityAfterRelease;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        isHolding = false;
    }

  // Gets the object's real collider bounds. Used for top/bottom and placement checks
    private bool TryGetObjectBounds(out Bounds bounds)
    {
        bounds = new Bounds(Vector3.zero, Vector3.zero);

        if (objectToPick == null)
            return false;

        Collider[] colliders = objectToPick.GetComponentsInChildren<Collider>();

        bool hasBounds = false;

        for (int i = 0; i < colliders.Length; i++)
        {
            Collider col = colliders[i];

            if (col == null || !col.enabled || col.isTrigger)
                continue;

            if (!hasBounds)
            {
                bounds = col.bounds;
                hasBounds = true;
            }
            else
            {
                bounds.Encapsulate(col.bounds);
            }
        }

        return hasBounds;
    }

  //-----------------------------------------------------------------------------------------------------------------
    private Transform GetActiveEndPoint()
    {
        if (graspPoint != null)
            return graspPoint;

        return robotTCP;
    }
   
   // Rotates one joint around its local axis
    private void RotateJoint(Transform joint, Vector3 axis, float angle)
    {
        if (joint == null)
            return;

        joint.Rotate(axis, angle, Space.Self);
    }

  //Normalises the joint angle to roughly -1 to 1, which is easier for the neural network
    private float GetNormalizedJointAngle(Transform joint, Vector3 axis)
    {
        return GetJointAngle(joint, axis) / 180f;
    }

    private float GetJointAngle(Transform joint, Vector3 axis)
    {
        if (joint == null)
            return 0f;

        Vector3 euler = joint.localEulerAngles;

        float angle;

        if (Mathf.Abs(axis.x) >= Mathf.Abs(axis.y) && Mathf.Abs(axis.x) >= Mathf.Abs(axis.z))
        {
            angle = euler.x;
        }
        else if (Mathf.Abs(axis.y) >= Mathf.Abs(axis.x) && Mathf.Abs(axis.y) >= Mathf.Abs(axis.z))
        {
            angle = euler.y;
        }
        else
        {
            angle = euler.z;
        }

        return NormalizeAngle(angle);
    }

    private float NormalizeAngle(float angle)
    {
        while (angle > 180f)
            angle -= 360f;

        while (angle < -180f)
            angle += 360f;

        return angle;
    }
  
  //Saves the initial pose, uses it to reset each episode
    private void SaveStartJointRotations()
    {
        if (jointA1 != null) startA1 = jointA1.localRotation;
        if (jointA2 != null) startA2 = jointA2.localRotation;
        if (jointA3 != null) startA3 = jointA3.localRotation;
        if (jointA4 != null) startA4 = jointA4.localRotation;
        if (jointA5 != null) startA5 = jointA5.localRotation;
        if (jointA6 != null) startA6 = jointA6.localRotation;
    }
  
  //Resets the joints to the initial pose at the start of each episode
    private void ResetJoints()
    {
        if (jointA1 != null) jointA1.localRotation = startA1;
        if (jointA2 != null) jointA2.localRotation = startA2;
        if (jointA3 != null) jointA3.localRotation = startA3;
        if (jointA4 != null) jointA4.localRotation = startA4;
        if (jointA5 != null) jointA5.localRotation = startA5;
        if (jointA6 != null) jointA6.localRotation = startA6;
    }

    private void ResetObject()
    {
        if (objectToPick == null)
            return;

        objectToPick.gameObject.SetActive(true);
        objectToPick.SetParent(objectStartParent, true);
        objectToPick.position = objectStartPosition;
        objectToPick.rotation = objectStartRotation;
        objectToPick.localScale = objectStartScale;

        Rigidbody rb = objectToPick.GetComponent<Rigidbody>();

        if (rb != null)
        {
            rb.isKinematic = false;
            rb.useGravity = true;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }


  //--------------------------------------------------------------------------------------------------------------
    private void CheckMaxSteps()
    {
        if (stepCount >= maxEpisodeSteps)
        {
            AddReward(-1.0f);
            EndEpisode();
        }
    }

 //-------------------------------------------------------------------------------------------------------------------
  //Visual debug drawing in the Scene view for placeArea, safePoint, forbidden area, etc
    private void OnDrawGizmosSelected()
    {
        if (placeArea != null)
        {
            Gizmos.color = Color.green;
            Gizmos.matrix = Matrix4x4.TRS(placeArea.position, placeArea.rotation, Vector3.one);
            Gizmos.DrawWireCube(Vector3.zero, placeAreaHalfExtents * 2f);

            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(Vector3.up * releaseHeight, 0.04f);

            Gizmos.color = Color.red;
            float surfaceY = placeAreaSurfaceOffset;
            Gizmos.DrawLine(new Vector3(-placeAreaHalfExtents.x, surfaceY, -placeAreaHalfExtents.z),
                            new Vector3(placeAreaHalfExtents.x, surfaceY, -placeAreaHalfExtents.z));
            Gizmos.DrawLine(new Vector3(placeAreaHalfExtents.x, surfaceY, -placeAreaHalfExtents.z),
                            new Vector3(placeAreaHalfExtents.x, surfaceY, placeAreaHalfExtents.z));
            Gizmos.DrawLine(new Vector3(placeAreaHalfExtents.x, surfaceY, placeAreaHalfExtents.z),
                            new Vector3(-placeAreaHalfExtents.x, surfaceY, placeAreaHalfExtents.z));
            Gizmos.DrawLine(new Vector3(-placeAreaHalfExtents.x, surfaceY, placeAreaHalfExtents.z),
                            new Vector3(-placeAreaHalfExtents.x, surfaceY, -placeAreaHalfExtents.z));

            Gizmos.matrix = Matrix4x4.identity;
        }

        if (startArea != null && placeArea != null)
        {
            Gizmos.color = Color.white;
            Gizmos.DrawLine(startArea.position, placeArea.position);
        }

        if (safePoint != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(safePoint.position, 0.06f);
        }

        if (robotStand != null)
        {
            Vector3 robotCenter = GetRobotStandBottomCenter();

            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(robotCenter, 0.06f);

            Gizmos.color = Color.black;
            Vector3 forbiddenSize = new Vector3(
                robotStandForbiddenSquareSize,
                0.02f,
                robotStandForbiddenSquareSize
            );

            Gizmos.DrawWireCube(
                new Vector3(robotCenter.x, robotCenter.y + 0.02f, robotCenter.z),
                forbiddenSize
            );

            if (safePoint != null)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(robotCenter, safePoint.position);
            }
        }

        if (targetPoint != null)
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawWireSphere(targetPoint.position, 0.06f);
        }
    }

  //------------------------------------------------------------------------------------------------------------
  //Keyboard control for manual testing
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = actionsOut.ContinuousActions;

        for (int i = 0; i < 6; i++)
        {
            actions[i] = 0f;
        }

        if (Input.GetKey(KeyCode.Alpha1)) actions[0] = 1f;
        if (Input.GetKey(KeyCode.Q)) actions[0] = -1f;

        if (Input.GetKey(KeyCode.Alpha2)) actions[1] = 1f;
        if (Input.GetKey(KeyCode.W)) actions[1] = -1f;

        if (Input.GetKey(KeyCode.Alpha3)) actions[2] = 1f;
        if (Input.GetKey(KeyCode.E)) actions[2] = -1f;

        if (Input.GetKey(KeyCode.Alpha4)) actions[3] = 1f;
        if (Input.GetKey(KeyCode.R)) actions[3] = -1f;

        if (Input.GetKey(KeyCode.Alpha5)) actions[4] = 1f;
        if (Input.GetKey(KeyCode.T)) actions[4] = -1f;

        if (Input.GetKey(KeyCode.Alpha6)) actions[5] = 1f;
        if (Input.GetKey(KeyCode.Y)) actions[5] = -1f;
    }
}