using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using PA_DronePack;
using Unity.VisualScripting;


public class DroneAgent : Agent
{   
    private PA_DroneController dcoscript;
    public GameObject[] goals;   // Reference to the delivery destination
    public GameObject[] boxes;    // Reference to the post (box)
    public GameObject start;
    float distanceToCurrentTarget;

    private Transform agentTrans;
    private Transform[] goalTrans;
    private Transform[] boxTrans;
    private Transform target;
    private Transform startTrans;

    private int totalBatteryUsed;
    private int batteryPerMove = 1;
    private int batteryPerLift = 2;
    private int batteryPerYaw = 1;
    private Rigidbody agent_Rigidbody;
    private Rigidbody box_Rigidbody;
    private Rigidbody goal_Rigidbody;

    private int maxCrashesAllowed = 5;
    private int crashCount;
    private bool isDelivering;
    private float previousDistance;
    private bool isReturning;
    private int randomIndex;
    public override void Initialize(){
        dcoscript = gameObject.GetComponent<PA_DroneController>();
        agentTrans = gameObject.transform;

        boxTrans = new Transform[boxes.Length];
        goalTrans = new Transform[goals.Length];
        startTrans = start.transform;
        
        for (int i = 0; i < boxes.Length; i++){
            boxTrans[i] = boxes[i].transform;
            goalTrans[i] = goals[i].transform;
        }

        //target = null; // hmm
        agent_Rigidbody = gameObject.GetComponent<Rigidbody>();

        isDelivering = false;
        isReturning = false;

        totalBatteryUsed = 0;
    }

    public override void CollectObservations(VectorSensor sensor){
        sensor.AddObservation(agentTrans.position - target.position);
        sensor.AddObservation(agent_Rigidbody.velocity);
        sensor.AddObservation(agent_Rigidbody.angularVelocity);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers){
        var actions = actionBuffers.ContinuousActions;

        float moveX = Mathf.Clamp(actions[0], -1f, 1f);
        float moveY = Mathf.Clamp(actions[1], -1f, 1f);
        float moveZ = Mathf.Clamp(actions[2], -1f, 1f);
        float yaw = Mathf.Clamp(actions[3], -1f, 1f);
        
        dcoscript.DriveInput(moveX);
        dcoscript.StrafeInput(moveY);
        dcoscript.LiftInput(moveZ);
        dcoscript.TurnInput(yaw);

        totalBatteryUsed += Mathf.Abs(moveX) > 0 ? batteryPerMove : 0;  // Movement in X direction
        totalBatteryUsed += Mathf.Abs(moveY) > 0 ? batteryPerMove : 0;  // Movement in Y direction
        totalBatteryUsed += Mathf.Abs(moveZ) > 0 ? batteryPerLift : 0;  // Vertical movement (lifting)
        totalBatteryUsed += Mathf.Abs(yaw) > 0 ? batteryPerYaw : 0;     // Yaw rotation

        float currentDistance = Vector3.Distance(target.position, agentTrans.position);
        float distanceDelta = previousDistance - currentDistance;
        if (distanceDelta > 0){
            AddReward(distanceDelta * 0.1f);
        }
        else{
            AddReward(distanceDelta * 0.2f);
        }
        
        if(agent_Rigidbody.velocity.magnitude < 0.1f){
            AddReward(-0.01f);
        }
        previousDistance = currentDistance;
    }

    public override void OnEpisodeBegin(){
        agentTrans.localPosition = new Vector3(8f, 1f, -8f);
        // boxTrans.localPosition = new Vector3(Random.Range(-8f, 0f), 1f, Random.Range(-8f, 0f));
        agentTrans.localRotation = Quaternion.Euler(0f, -45f, 0f);
        foreach (var box in boxes){
            box.SetActive(false);
        }
    
        foreach (var goal in goals){
            goal.SetActive(false);
        }
        
        randomIndex = Random.Range(0, boxes.Length);
        boxes[randomIndex].SetActive(true); 
        goals[randomIndex].SetActive(true);
        target = boxTrans[randomIndex]; 

        isDelivering = false; // Agent should start heading towards the box

        isReturning = false;

        totalBatteryUsed = 0;
        
        crashCount = 0;

        previousDistance = Vector3.Distance(agentTrans.position, boxTrans[randomIndex].position);
    }


    public void OnCollisionEnter(Collision coll)
    {
        if (coll.collider.CompareTag("BOUND"))  // Out of bounds
        {
            Debug.Log("Out of Bounds");
            AddReward(-0.01f);  // Penalty for going out of bounds
            var statsRecorder = Academy.Instance.StatsRecorder;
            statsRecorder.Add("Drone/Total Battery Used", totalBatteryUsed);
            EndEpisode();
        }
        else if (coll.collider.CompareTag("WALL") && !isReturning)  // Collision with walls or other obstacles
        {
            Debug.Log("Crash with APT");
            AddReward(-0.1f);  // Penalty for crashing into walls
            var statsRecorder = Academy.Instance.StatsRecorder;
            statsRecorder.Add("Drone/Total Battery Used", totalBatteryUsed);
            EndEpisode();
        }
        else if (coll.collider.CompareTag("WALL") && isReturning)
        {
            Debug.Log("Crash while returning");
            AddReward(-0.01f);
            crashCount++;
            if(crashCount >= maxCrashesAllowed)
            {
                Debug.Log("Too many crashes");
                AddReward(-1f);
                EndEpisode();
            }
        }
        else if (coll.collider.CompareTag("POST"))  // Picking up the post
        {
            Debug.Log("Picked up post");
            AddReward(5f);  // Reward for picking up the post
            boxes[randomIndex].SetActive(false);  // Deactivate the box object after it's picked up
            isDelivering = true;  // Now the agent can head to the destination
            target = goalTrans[randomIndex];
            previousDistance = Vector3.Distance(agentTrans.position, goalTrans[randomIndex].position); // Reset distance tracking to the destination
        }
        else if (coll.collider.CompareTag("DEST") && isDelivering)  // Reaching the destination
        {
            goals[randomIndex].SetActive(false);
            Debug.Log("Success");
            SetReward(10f);  // Large reward for successfully delivering the post
            //EndEpisode();
            isReturning = true;
            target = startTrans;
            isDelivering = false;
        }
        else if (coll.collider.CompareTag("DEST") && !isDelivering)  // Reaching the destination before picking up the post
        {
            Debug.Log("Reached destination without picking up the post - Penalty");
            AddReward(-1f);  // Large penalty for reaching the destination without the post
            //EndEpisode();
        }
        else if (coll.collider.CompareTag("START") && !isReturning)
        {
        }
        else if (coll.collider.CompareTag("START") && isReturning)
        {
            Debug.Log("Returned Successfully");
            AddReward(15f);
            var statsRecorder = Academy.Instance.StatsRecorder;
            statsRecorder.Add("Drone/Total Battery Used", totalBatteryUsed);
            EndEpisode();
        }
    }
}
