using System;
using UnityEngine;

using BDArmory.Competition;
using BDArmory.CounterMeasure;
using BDArmory.Extensions;
using BDArmory.Radar;
using BDArmory.Settings;
using BDArmory.Utils;

namespace BDArmory.Targeting
{
    public struct TargetSignatureData : IEquatable<TargetSignatureData>
    {
        public Vector3 velocity;
        public Vector3 geoPos;
        public Vector3 _position;
        public Vector3 acceleration;
        public bool exists;
        public float timeAcquired;
        public float signalStrength;
        public TargetInfo targetInfo;
        public BDTeam Team;
        public Vector2 pingPosition;
        public VesselECMJInfo vesselJammer;
        public ModuleRadar lockedByRadar;
        public Vessel vessel;
        public Part IRSource;
        public bool orbital;
        Orbit orbit;

        public bool Equals(TargetSignatureData other)
        {
            return
                exists == other.exists &&
                geoPos == other.geoPos &&
                timeAcquired == other.timeAcquired;
        }

        public TargetSignatureData(Vessel v, float _signalStrength, Part heatpart = null)
        {
            orbital = v.InOrbit();
            orbit = v.orbit;

            timeAcquired = Time.time;
            vessel = v;
            velocity = v.Velocity();
            IRSource = heatpart;
            position = IRSource != null ?
                IRSource.transform.position :
                v.CoM;
            acceleration = v.acceleration_immediate;
            exists = true;

            signalStrength = _signalStrength;

            targetInfo = v.gameObject.GetComponent<TargetInfo>();

            // vessel never been picked up on radar before: create new targetinfo record
            if (targetInfo == null)
            {
                targetInfo = v.gameObject.AddComponent<TargetInfo>();
            }

            Team = null;

            if (targetInfo)  // Always true, as we just set it?
            {
                Team = targetInfo.Team;
            }
            else
            {
                var mf = VesselModuleRegistry.GetMissileFire(v, true);
                if (mf != null) Team = mf.Team;
            }

            vesselJammer = v.gameObject.GetComponent<VesselECMJInfo>();

            pingPosition = Vector2.zero;
            lockedByRadar = null;
        }

        public TargetSignatureData(CMFlare flare, float _signalStrength)
        {
            velocity = flare.velocity;
            position = flare.transform.position;
            exists = true;
            acceleration = Vector3.zero;
            timeAcquired = Time.time;
            signalStrength = _signalStrength;
            targetInfo = null;
            vesselJammer = null;
            Team = null;
            pingPosition = Vector2.zero;
            orbital = false;
            orbit = null;
            lockedByRadar = null;
            vessel = null;
            IRSource = null;
        }

        public TargetSignatureData(Vector3 _velocity, Vector3 _position, Vector3 _acceleration, bool _exists, float _signalStrength)
        {
            velocity = _velocity;
            position = _position;
            acceleration = _acceleration;
            exists = _exists;
            timeAcquired = Time.time;
            signalStrength = _signalStrength;
            targetInfo = null;
            vesselJammer = null;
            Team = null;
            pingPosition = Vector2.zero;
            orbital = false;
            orbit = null;
            lockedByRadar = null;
            vessel = null;
            IRSource = null;
        }

        public Vector3 position
        {
            get {
                return orbital ?
                    _position :
                    VectorUtils.GetWorldSurfacePostion(geoPos, FlightGlobals.currentMainBody);
            }
            set
            {
                geoPos = VectorUtils.WorldPositionToGeoCoords(value, FlightGlobals.currentMainBody);
                _position = value;
            }
        }

        public Vector3 predictedPosition
        {
            get
            {
                return position + (velocity * age);
            }
        }

        public (Vector3, Vector3, Vector3) TargetDataWithChaffFactor(float chaffEffectivity = 1f) {
            // get chaff factor of vessel and calculate decoy distortion caused by chaff echos
            float decoyFactor = 0f;
            Vector3 posDistortion = Vector3.zero;
            Vector3 velDistortion = Vector3.zero;
            Vector3 accDistortion = Vector3.zero;

            if (vessel != null)
            {
                // chaff check
                decoyFactor = (1f - RadarUtils.GetVesselChaffFactor(vessel));
                Debug.Log($"[BDArmory.Chaff]: Target decoy factor: {decoyFactor}");

                if (decoyFactor > 0f)
                {
                    // With ecm on better chaff effectiveness due to jammer strength
                    VesselECMJInfo vesseljammer = vessel.gameObject.GetComponent<VesselECMJInfo>();

                    // Jamming biases position distortion further to rear, depending on ratio of jamming strength and radarModifiedSignature
                    float jammingFactor = vesseljammer is null ? 0 : decoyFactor * Mathf.Clamp01(vesseljammer.jammerStrength / 100f / Mathf.Max(targetInfo.radarModifiedSignature, 0.1f));
                    
                    float chaffFactor = Mathf.Max(BDArmorySettings.CHAFF_FACTOR, 0f);

                    if (orbital) {
                        const float distortionMaxDistance = 256f;
                        // Semi-random vector, continuous with time, and its derivatives
                        var (dp, dv, da) = WonderingVector(Time.fixedTime,BDArmorySettings.CHAFF_DISTORTION_SPEED);
                        
                        float distortionFactor = decoyFactor * chaffEffectivity * chaffFactor * distortionMaxDistance;
                        
                        posDistortion = dp * distortionFactor;
                        velDistortion = dv * distortionFactor * jammingFactor;
                        accDistortion = da * distortionFactor * jammingFactor * jammingFactor;
                    }
                    else {
                        // Random radius of distortion, 16-256m
                        float distortionFactor = decoyFactor * UnityEngine.Random.Range(16f, 256f);
                        
                        // Convert Float jammingFactor position bias and signatureFactor scaling to Vector3 position
                        Vector3 signatureDistortion = distortionFactor *
                            (vessel.GetSrfVelocity().normalized * -1f * jammingFactor + UnityEngine.Random.insideUnitSphere);

                        // Higher speed -> missile decoyed further "behind" where the chaff drops (also means that chaff is least effective for head-on engagements)
                        posDistortion = (vessel.GetSrfVelocity() * -1f * Mathf.Clamp(decoyFactor * decoyFactor, 0f, 0.5f)) +
                            signatureDistortion;

                        // Apply effects from global settings and individual missile chaffEffectivity
                        posDistortion *= chaffFactor * chaffEffectivity;
                    }
                }
            }

            return (
                position + (velocity * age) + (acceleration * age * age / 2) + posDistortion,
                velocity + (acceleration * age) + velDistortion,
                acceleration + accDistortion
            );
        }

        (Vector3, Vector3, Vector3) WonderingVector(float t, float speed = 1f) {
            var x = VectorUtils.FullPerlinNoiseWithDerivatives(
                t * speed,
                0,
                Time.fixedDeltaTime * speed
            );
            
            var y = VectorUtils.FullPerlinNoiseWithDerivatives(
                t * speed,
                50,
                Time.fixedDeltaTime * speed
            );
            
            var z = VectorUtils.FullPerlinNoiseWithDerivatives(
                t * speed,
                100,
                Time.fixedDeltaTime * speed
            );

            return (new Vector3(x.Item1, y.Item1, z.Item1),
                new Vector3(x.Item2, y.Item2, z.Item2) * speed,
                new Vector3(x.Item3, y.Item3, z.Item3) * speed);
        }

        public float altitude
        {
            get
            {
                return geoPos.z;
            }
        }

        public float age
        {
            get
            {
                return (Time.time - timeAcquired);
            }
        }

        public static TargetSignatureData noTarget
        {
            get
            {
                return new TargetSignatureData(Vector3.zero, Vector3.zero, Vector3.zero, false, (float)RadarWarningReceiver.RWRThreatTypes.None);
            }
        }

        public static void ResetTSDArray(ref TargetSignatureData[] tsdArray)
        {
            for (int i = 0; i < tsdArray.Length; i++)
            {
                tsdArray[i] = noTarget;
            }
        }
    }
}
