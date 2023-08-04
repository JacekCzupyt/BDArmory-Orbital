using System;
using BDArmory.Weapons.Missiles;
using MathNet.Numerics;
using UniLinq;
using UnityEngine;

namespace BDArmory.Guidances; 

public class VacuumGuidance : IGuidance {
    private readonly MissileLauncher _launcher;
    //TODO: replace launcher with alternate method of obtaining MaxThrust
    public VacuumGuidance(MissileLauncher launcher) {
        _launcher = launcher;
    }
    public Vector3 GetDirection(MissileBase missile, Vector3 targetPosition, Vector3 targetVelocity, Vector3 targetAcceleration) {

        Vector3? desiredAcceleration;
        float maxMissileAcceleration = (float)(_launcher.thrust / missile.vessel.totalMass);
        if (!missile.TargetAcquired) {
            desiredAcceleration = missile.transform.forward * maxMissileAcceleration;
        }
        else {
            (desiredAcceleration, _) = ComputeVacuumIntercept(
                targetPosition - missile.transform.position,
                targetVelocity - missile.vessel.obt_velocity,
                targetAcceleration - missile.vessel.graviticAcceleration,
                maxMissileAcceleration
            );
            if (!desiredAcceleration.HasValue) {
                // If no solution found, disregard target acceleration
                (desiredAcceleration, _) = ComputeVacuumIntercept(
                    targetPosition - missile.transform.position,
                    targetVelocity - missile.vessel.obt_velocity,
                    Vector3.zero,
                    maxMissileAcceleration
                );
            }
        }
        if (!desiredAcceleration.HasValue) throw new Exception("Desired acceleration not found");

        return desiredAcceleration.Value;
    }
    
    static (Vector3?, float) ComputeVacuumIntercept(
        Vector3 targetPosition,
        Vector3 targetVelocity,
        Vector3 targetAcceleration,
        float maxMissileAcceleration
    ) {
        var equation = new Polynomial(-maxMissileAcceleration * maxMissileAcceleration);
        for (int i = 0; i < 3; i++) {
            var axisPolynomial = new Polynomial(
                targetAcceleration[i],
                2 * targetVelocity[i],
                2 * targetPosition[i]
            );
            equation += axisPolynomial * axisPolynomial;
        }
        var roots = equation.Roots().Where(e => e.IsReal()).Select(e => e.Real).ToArray();
        
        if (!roots.Any())
            return (null, float.PositiveInfinity);
        
        float targetTime = (float)(1 / roots.Max());
        
        var missileAcceleration = 2 * targetPosition / (targetTime * targetTime) + 2 * targetVelocity / targetTime + targetAcceleration;
        
        return (missileAcceleration, targetTime);
    }
}
