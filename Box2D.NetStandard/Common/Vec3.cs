

using Box2D.Common;

namespace Box2D.NetStandard.Common
{
    /// <summary>
    /// A 2D column vector with 3 elements
    /// </summary>
    public struct Vec3
    {
        /// <summary>
        /// Construct using coordinates
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        public Vec3(float x, float y, float z) { X = x; Y = y; Z = z; }

        /// <summary>
        /// Set this vector to all zeros.
        /// </summary>
        public void SetZero() { X = 0.0f; Y = 0.0f; Z = 0.0f; }

        /// <summary>
        /// Set this vector to some specified coordinates.
        /// </summary>
        public void Set(float x, float y, float z) { X = x; Y = y; Z = z; }

        /// <summary>
        /// Perform the dot product on two vectors.
        /// </summary>
        public static float Dot(Vec3 a, Vec3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        /// <summary>
        /// Perform the cross product on two vectors.
        /// </summary>
        public static Vec3 Cross(Vec3 a, Vec3 b)
        {
            return new Vec3(a.Y * b.Z - a.Z * b.Y,  a.Z *  b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X );
        }

        /// <summary>
        /// Negate this vector.
        /// </summary>
        public static Vec3 operator -(Vec3 v)
        {
            return new Vec3(-v.X, -v.Y, -v.Z);
        }

        /// <summary>
        /// Add two vectors component-wise.
        /// </summary>
        public static Vec3 operator +(Vec3 v1, Vec3 v2)
        {
            return new Vec3(v1.X  + v2.X, v1.Y + v2.Y,  v1.Z + v2.Z);
        }

        /// <summary>
        /// Subtract two vectors component-wise.
        /// </summary>
        public  static Vec3 operator -(Vec3 v1,  Vec3 v2)
        {
            return new Vec3(v1.X  - v2.X, v1.Y - v2.Y, v1.Z  - v2.Z);
        }

        /// <summary>
        /// Multiply this vector by a scalar.
        /// </summary>
        public static Vec3 operator *(Vec3 v, float s) {

            return new Vec3(v.X * s, v.Y * s, v.Z * s);
        }

        /// <summary>
        /// Multiply this vector by a scalar.
        /// </summary>
        public static Vec3 operator *(float s, Vec3 v)
        {
            return new Vec3(v.X * s, v.Y * s, v.Z * s);
        }


        public float X, Y, Z;

    }

}
