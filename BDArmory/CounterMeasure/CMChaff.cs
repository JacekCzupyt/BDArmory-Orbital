using System.Collections;
using UnityEngine;

using BDArmory.Utils;

namespace BDArmory.CounterMeasure
{
    public class CMChaff : MonoBehaviour
    {
        KSPParticleEmitter pe;

        const float drag = 5;

        Vector3d position;
        Vector3 velocity;
        CelestialBody body;

        public void Emit(Vector3 position, Vector3 velocity)
        {
            transform.position = position;
            this.velocity = velocity;
            gameObject.SetActive(true);
        }

        void OnEnable()
        {
            if (!pe)
            {
                pe = gameObject.GetComponentInChildren<KSPParticleEmitter>();
                pe.useWorldSpace = false;
                EffectBehaviour.AddParticleEmitter(pe);
            }

            body = FlightGlobals.currentMainBody;
            if (!body)
            {
                gameObject.SetActive(false);
                return;
            }

            StartCoroutine(LifeRoutine());
        }

        void OnDisable()
        {
            body = null;
        }

        IEnumerator LifeRoutine()
        {
            position = transform.position; // Optimisation: avoid getting/setting transform.position more than necessary.
            pe.EmitParticle();

            float startTime = Time.time;
            var wait = new WaitForFixedUpdate();

            while (Time.time - startTime < pe.maxEnergy)
            {
                velocity += FlightGlobals.getGeeForceAtPosition(position) * Time.fixedDeltaTime;
                Vector3 surfVelocity = velocity - (Vector3)body.getRFrmVel(position);
                Vector3 dragForce = (0.008f) *
                    drag *
                    0.5f *
                    surfVelocity.sqrMagnitude *
                    (float)
                    FlightGlobals.getAtmDensity(
                        FlightGlobals.getStaticPressure(position),
                        FlightGlobals.getExternalTemperature(),
                        body
                    ) *
                    surfVelocity.normalized;
                velocity -= (dragForce) * Time.fixedDeltaTime;

                if (body.GetAltitude(position) > 100000)
                    position += velocity * Time.fixedDeltaTime;
                else
                    position += surfVelocity * Time.fixedDeltaTime;

                if (BDKrakensbane.IsActive)
                    position -= BDKrakensbane.FloatingOriginOffsetNonKrakensbane;
                
                transform.position = position;
                yield return wait;
            }

            gameObject.SetActive(false);
        }
    }
}
