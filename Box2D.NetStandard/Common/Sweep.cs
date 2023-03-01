
using Box2D.Common;

namespace Box2D.NetStandard.Common
{
    public struct Sweep
    {
        public Vec2 LocalCenter; // local  center of mass position
        public Vec2 C0, C;       //  local center  of mass position
        public float A0, A;      //  world angles
        public float T0;         // time interval = [T0,1], where T0 is in [0,1]

        public void GetTransform(out XForm xf, float  alpha)
        {
            xf = new XForm();
            xf.Position = (1.0f - alpha) * C0 + alpha * C;
            float angle = (1.0f - alpha) * A0 + alpha * A;
            xf.R.Set(angle);

            // Shift to origin
            xf.Position -= Common.Math.Mul(xf.R, LocalCenter);
        }

        public void Advance(float t)
        {
            if (T0 < t && 1.0f - T0 >  Settings.FLT_EPSILON)
            {
                float alpha = (t - T0) / (1.0f - T0);
                C0 = (1.0f - alpha) * C0 + alpha * C;
                A0 = (1.0f - alpha) * A0 + alpha * A;
                T0 = t;
            }
        }
    }
}
