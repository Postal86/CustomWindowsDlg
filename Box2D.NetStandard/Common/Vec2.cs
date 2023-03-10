

using Box2D.NetStandard.Common;
using Box2DX;
using System.ComponentModel;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.Common
{
    /// <summary>
    /// A 2D column vector
    /// </summary>
    public struct Vec2
    {
        public float X, Y;

        public float this[int i]
        {
            get
            {
                if (i == 0) return X;
                else if (i == 1) return Y;
                else
                {
                    Box2DXDebug.Assert(false, "Incorrect Vec2 element!");
                    return 0; 
                }
            }
            set
            {
                if (i == 0) X = value;
                else if (i == 1) Y = value;
                else
                {
                    Box2DXDebug.Assert(false, "Incorrect Vec2  element!");
                }
            }
        }

        /// <summary>
        ///  Construct  using coordinates.
        /// </summary>
        
        public Vec2(float x)
        {
            X = x;
            Y = x;
        }


        /// <summary>
        ///  Construct  using coordinates.
        /// </summary>
        public Vec2(float x, float y)
        {
            X = x;
            Y = y;
        }


        /// <summary>
        /// Set this vector to all zeros.
        /// </summary>
        public void SetZero() { X = 0.0f; Y = 0.0f; }

        /// <summary>
        ///  Set this  vector  to some specified coordinates.
        /// </summary>
        
        public void Set(float x, float y) { X = x; Y = y; }

        public void Set(float xy) { X = xy; Y = xy; }

        /// <summary>
        /// 
        /// </summary>
        
        public float Length()
        {
            return (float)System.Math.Sqrt(X * X + Y * Y);
        }

        /// <summary>
        /// Get the length  squared. For performance, use this  instead of
        /// Length (if  possible)
        /// </summary>
        /// <returns></returns>
        public float LengthSquared()
        {
            return X * X + Y * Y;
        }

        /// <summary>
        /// Convert this vector into a  unit vector. Returns the length
        /// </summary>
        /// <returns></returns>
        public float Normalize()
        {
            float length = Length();
            if (length < Settings.FLT_EPSILON)
            {
                return 0.0f;
            
            }
            
            float invLength = 1.0f / length;
            X *= invLength;
            Y *= invLength;

            return length; 

        }

        /// <summary>
        /// Does  this vector contain finite  coordinates?
        /// </summary>
        public bool IsValid
        {
            get { return Math.IsValid(X) && Math.IsValid(Y); }
        }

        /// <summary>
        /// Negate this vector.
        /// </summary>
        public  static Vec2  operator -(Vec2 v1)
        {
            Vec2 v = new Vec2();
            v.Set(-v1.X, -v1.Y);
            return v;
        }

        public static Vec2  operator +(Vec2 v1, Vec2 v2)
        {
            Vec2 v = new Vec2();
            v.Set(v1.X + v2.X, v1.Y + v2.Y);
            return v;
        }

        public static Vec2 operator -(Vec2 v1, Vec2 v2)
        {
            Vec2 v = new Vec2();
            v.Set(v1.X - v2.X, v1.Y - v2.Y);
            return v;
        }

        public static Vec2 operator *(Vec2 v1, float a)
        {
            Vec2 v = new Vec2();
            v.Set(v1.X * a, v1.Y * a);
            return v;
        }

        public static Vec2 operator *(float a, Vec2 v1)
        {
            Vec2 v = new Vec2();
            v.Set(v1.X * a, v1.Y * a);
            return v;
        }

        public static bool  operator ==(Vec2 a,  Vec2 b)
        {
            return a.X  == b.X && a.Y == b.Y;
        }

        public static bool operator !=(Vec2 a, Vec2 b)
        {
            return a.X != b.X || a.Y != b.Y;
        }

        public static Vec2 Zero { get { return new Vec2(0, 0); } }


        /// <summary>
        /// Peform the dot product on two vectors.
        /// </summary>
        public static float Dot(Vec2 a,  Vec2 b)
        {
            return a.X  *  b.Y - a.Y * b.X;
        }

        /// <summary>
        /// Perform the cross product on two vectors. In 2D this produces a scalar.
        /// </summary>
        public static float Cross(Vec2 a,  Vec2 b)
        {
            return a.X * b.Y - a.Y * b.X;
        }

        /// <summary>
        /// Perform the cross product on a vector and a scalar. 
        /// In 2D this produces a vector.
        /// </summary>
        public static Vec2 Cross(Vec2 a, float s)
        {
            Vec2 v = new Vec2();
            v.Set(s * a.Y, -s * a.X);
            return v;
        }

        /// <summary>
        /// Perform the cross product on a scalar and a vector. 
        /// In 2D this produces a vector.
        /// </summary>
        public static Vec2 Cross(float s, Vec2 a)
        {
            Vec2 v = new Vec2();
            v.Set(-s * a.Y,  s * a.X);
            return v;
        }

        public  static float Distance(Vec2 a, Vec2 b)
        {
            Vec2 c = a - b;
            return c.Length();

        }

        public static float DistanceSquared(Vec2 a,  Vec2 b)
        {
            Vec2 c = a - b;
            return Vec2.Dot(c, c);
        }

    }
}
