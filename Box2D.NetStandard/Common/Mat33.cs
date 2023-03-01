using Box2D.Common;
using Box2DX;

namespace Box2D.NetStandard.Common
{

    /// <summary>
    /// A 3-by-3 matrix. Stored in column-major order.
    /// </summary>
    public struct Mat33
    {
        /// <summary>
        /// Construct this matrix using columns.
        /// </summary>
        public Mat33(Vec3 c1, Vec3  c2, Vec3  c3)
        {
            Col1 = c1;
            Col2 = c2;
            Col3 = c3;
        }

        /// <summary>
        /// Set this matrix to all zeros.
        /// </summary>
        public void SetZero()
        {
            Col1.SetZero();
            Col2.SetZero();
            Col3.SetZero();
        }


        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        /// </summary>
        public Vec2  Solve22(Vec2  b)
        {
            float a11 = Col1.X, a12 = Col2.X, a21 = Col1.Y, a22 = Col2.Y;
            float det = a11 * a22 - a12 * a21;
            Box2DXDebug.Assert(det  != 0.0f);
            det = 1.0f / det;
            Vec2 x = new Vec2();
            x.X = det * (a22 * b.X - a12 * b.Y);
            x.Y = det * (a11 * b.Y - a21 * b.X);
            return x; 
        }


        public Vec3 Col1, Col2, Col3;
    }

    
}
