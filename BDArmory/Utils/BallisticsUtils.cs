using System;
using System.Collections.Generic;
using BDArmory.Extensions;
using BDArmory.Settings;
using UnityEngine;

namespace BDArmory.Utils; 

public class BallisticsUtils {
    private Part Part { get; }
    private readonly int _layerMask;
    private readonly List<Vector3> trajectoryPoints;
    private readonly bool drawLines = false;
    private readonly Vector3 activeVesselVelocity = Vector3.zero;
    private readonly Vector3 activeVesselAcceleration = Vector3.zero;
    private float elapsedTime = 0f;
    private readonly bool visualRelativeToVessel = false;

    [Flags]
    public enum SimulationStage { Normal, Refining, Final };

    public BallisticsUtils(int layerMask, Part part, List<Vector3> trajectoryPoints = null, Transform fireTransform = null) {
        Part = part;
        _layerMask = layerMask;
        this.trajectoryPoints = trajectoryPoints;
        
        if (BDArmorySettings.DEBUG_LINES && BDArmorySettings.DRAW_AIMERS && trajectoryPoints != null) {
            drawLines = true;
            trajectoryPoints.Clear();
            if(fireTransform != null)
                trajectoryPoints.Add(fireTransform.position);
            
            var activeVessel = FlightGlobals.ActiveVessel;
            if (activeVessel.InVacuum()) {
                visualRelativeToVessel = true;
                activeVesselVelocity = activeVessel.Velocity();
                activeVesselAcceleration = activeVessel.acceleration;
            }
        }
    }
    
    /// <summary>
    /// Use the leapfrog numerical integrator for a ballistic trajectory simulation under the influence of just gravity.
    /// The leapfrog integrator is a second-order symplectic method.
    /// 
    /// Note: Use this to see the trajectory with collision detection, but use BallisticTrajectoryClosestApproachSimulation instead for targeting purposes.
    /// </summary>
    /// <param name="position"></param>
    /// <param name="velocity"></param>
    /// <param name="maxTime"></param>
    /// <param name="timeStep"></param>
    public float BallisticTrajectorySimulation(ref Vector3 position, Vector3 velocity, float maxTime, float timeStep, bool ignoreWater = false, SimulationStage stage = SimulationStage.Normal)
    {
        if (FlightGlobals.getAltitudeAtPos(position) < 0) ignoreWater = true;
        var gravity = (Vector3)FlightGlobals.getGeeForceAtPosition(position);
        velocity += 0.5f * timeStep * gravity; // Boot-strap velocity calculation.
        Ray ray = new Ray();
        if (drawLines) {
            trajectoryPoints.Add(position);
        }
        while (elapsedTime < maxTime)
        {
            ray.origin = position;
            ray.direction = velocity;
            var deltaPosition = timeStep * velocity;
            var deltaDistance = deltaPosition.magnitude;
            var altitude = FlightGlobals.getAltitudeAtPos(position + deltaPosition);

            if (stage == SimulationStage.Normal && elapsedTime + timeStep > maxTime) { // Out of time
                // Debug.Log($"DEBUG Refining trajectory sim due to final time, time: {elapsedTime}, {timeStep}, {maxTime}, dist: {maxDistance}, {elapsedDistance}, {deltaDistance}");
                velocity -= 0.5f * timeStep * gravity; // Correction to final velocity.
                BallisticTrajectorySimulation(ref position, velocity, maxTime, (maxTime - elapsedTime) / 4f, ignoreWater, SimulationStage.Final);
                break;
            }

            RaycastHit hit;
            if ((Physics.Raycast(ray, out hit, deltaDistance, _layerMask) && (hit.collider != null && hit.collider.gameObject != null && hit.collider.gameObject.GetComponentInParent<Part>() != Part)) // Ignore the part firing the projectile.
                || (!ignoreWater && altitude < 0)) // Underwater
            {
                switch (stage)
                {
                    case SimulationStage.Refining | SimulationStage.Normal: // Perform a more accurate final step for the collision.
                        {
                            // Debug.Log($"DEBUG Refining trajectory sim, time: {elapsedTime}, {timeStep}, {maxTime}, dist: {maxDistance}, {elapsedDistance}, {deltaDistance}");
                            velocity -= 0.5f * timeStep * gravity; // Correction to final velocity.
                            BallisticTrajectorySimulation(ref position, velocity, elapsedTime + timeStep, timeStep / 4f, ignoreWater, timeStep > 5f * Time.fixedDeltaTime ? SimulationStage.Refining : SimulationStage.Final);
                            break;
                        }
                    case SimulationStage.Final:
                        {
                            if (!ignoreWater && altitude < 0) // Underwater
                            {
                                var currentAltitude = FlightGlobals.getAltitudeAtPos(position);
                                timeStep *= currentAltitude / (currentAltitude - altitude);
                                elapsedTime += timeStep;
                                position += timeStep * velocity;
                                // Debug.Log("DEBUG breaking trajectory sim due to water at " + position.ToString("F6") + " at altitude " + FlightGlobals.getAltitudeAtPos(position));
                            }
                            else // Collision
                            {
                                elapsedTime += (hit.point - position).magnitude / velocity.magnitude;
                                position = hit.point;
                                // Debug.Log("DEBUG breaking trajectory sim due to hit at " + position.ToString("F6") + " at altitude " + FlightGlobals.getAltitudeAtPos(position));
                            }
                            break;
                        }
                }
                break;
            }

            position += deltaPosition;
            gravity = FlightGlobals.getGeeForceAtPosition(position);
            velocity += timeStep * gravity;
            elapsedTime += timeStep;
            if (BDArmorySettings.DEBUG_LINES && BDArmorySettings.DRAW_AIMERS && stage != SimulationStage.Final) {
                var visualOffset = visualRelativeToVessel ?
                    AIUtils.PredictPosition(Vector3.zero, activeVesselVelocity, activeVesselAcceleration, elapsedTime) :
                    Vector3.zero;
                trajectoryPoints.Add(position - visualOffset);
            }
        }
        return elapsedTime;
    }
}
