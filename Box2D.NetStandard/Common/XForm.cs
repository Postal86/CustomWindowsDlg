using Box2D.Common;

namespace Box2D.NetStandard.Common
{
    /// <summary>
    /// A transform contains translation and rotation.
    /// It is used to represent the position and orientation of rigid frames.
    /// </summary>
    public struct XForm
    {
        public Vec2 Position;
        public Mat22 R;

        /// <summary>
        /// Initialize using a position vector and a rotation matrix.
        /// </summary>
        /// <param name="position"></param>
        /// <param name="R"></param>
        public XForm(Vec2 position, Mat22 rotation)
        {
            Position= position;
            R= rotation;
        }

        /// <summary>
        /// Set this to the identity transform.
        /// </summary>
        public void SetIdentity()
        {
            Position.SetZero();
            R.SetIdentity();
        }

        /// Set this based on the position and angle.
        public void Set(Vec2 p, float angle)
        {
            Position = p;
            R.Set(angle);
        }

        /// Calculate the angle that the rotation matrix represents.
        public float GetAngle()
        {
            return Math.Atan2(R.Col1.Y, R.Col1.X);
        }

        public static XForm Identity { get { return new XForm(Vec2.Zero, Mat22.Identity); } }
    }

}
