using System;
using Box2D.Common;

namespace Box2D.NetStandard.Collision
{
    // Structures  and functions used for computing contacts points, distance
    // Queries and  TOI queries
    public partial class Collision
    {
        public static readonly byte NullFeature = Common.Math.UCHAR_MAX;

        public static  bool TestOverlap(AABB a,  AABB b)
        {
            Vec2 d1, d2;
            d1 = b.LowerBound - a.UpperBound;
            d2 = a.LowerBound - b.UpperBound;

            if (d1.X > 0.0f || d1.Y > 0.0f)
                return false;

            if (d2.X > 0.0f || d2.Y > 0.0f)
                return false;

            return true;
        }

        /// <summary>
        /// Compute the point states given two manifolds. The states pertain to the transition from manifold1
        /// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
        /// </summary>
        public static  void GetPointStates(PointState[/*b2_maxManifoldPoints*/]  state1, PointState[/*b2_maxManifoldPoints*/] state2, 
            Manifold manifold1, Manifold manifold2)
        {
            for (int i = 0; i < Settings.MaxManifoldPoints; ++i)
            {
                state1[i] = PointState.NullState;
                state1[i] = PointState.NullState;
            }

            // Detect  persists  and  removes
            for (int i = 0; i < manifold1.PointCount; ++i)
            {
                ContactID id = manifold1.Points[i].ID;

                state1[i] = PointState.RemoveState;

                // Detect persists and removes.
                for (int i = 0; i < manifold1.PointCount; ++i)
                {
                    ContactID id = manifold1.Points[i].ID;

                    state1[i] = PointState.RemoveState;

                    for (int j = 0; j < manifold2.PointCount; ++j)
                    {
                        if (manifold2.Points[j].ID.Key == id.Key)
                        {
                            state1[i] = PointState.PersistState;
                            break;
                        }

                    }
                }
            }

                // Detect persists  and adds
                for (int i = 0; i < manifold2.PointCount; ++i)
                {
                  ContactID id = manifold2.Points[i].ID;

                  state2[i] = PointState.AddState; 

                  for (int j =  0; j <  manifold1.PointCount; ++j)
                  {
                    if (manifold1.Points[j].ID.Key == id.Key)
                    {
                        state2[i] = PointState.PersistState;
                        break;
                    }
                  
                  }
                
                }
            }

         // Sutherland-Hodgman clipping.
        public static int ClipSegmentToLine(out ClipVertex[] vOut, ClipVertex  vIn, Vec2  normal, float offset)
        {
            vOut = new ClipVertex[2];

            // Start with no output  points 
            int numOut = 0;


            // Calculate the  distance  of end points  to the  line 
            float distance0 = Vec2.Dot(normal, vIn[0].V) - offset;
            float distance1 = Vec2.Dot(normal, vIn[1].V) - offset;

            // If the  points are behind the plane
            if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
            if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

            // If the  points are on  different  sides  of the plane
            if (distance0 *  distance1 < 0.0f)
            {
                // Find intersection point of edge  and plane
                float interp = distance0 / (distance0 - distance1);
                vOut[numOut].V = vIn[0].V + interp * (vIn[1] - vIn[0].V);
                if (distance0 > 0.0f)
                {
                    vOut[numOut].ID = vIn[0].ID;
                }
                else
                {
                    vOut[numOut].ID = vIn[1].ID;
                }
                ++numOut;
            }

            return numOut;
        }

    }

    /// <summary>
	/// The features that intersect to form the contact point.
	/// </summary>
    public struct Features
    {

        /// <summary>
        /// The edge that defines the outward contact normal.
        /// </summary>
        public Byte ReferenceEdge;

        /// <summary>
        /// The edge most anti-parallel to the reference edge.
        /// </summary>
        public Byte IncidentEdge;

        /// <summary>
        /// The vertex (0 or 1) on the incident edge that was clipped.
        /// </summary>
        public Byte IncidentVertex;

        /// <summary>
        /// A value of 1 indicates that the reference edge is on shape2.
        /// </summary>
        public Byte Flip;

    }


    /// <summary>
    /// Contact ids to facilitate warm starting.
    /// </summary>
    [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
    public struct ContactID
    {
        [System.Runtime.InteropServices.FieldOffset(0)]
        public Features Features;

        /// <summary>
        /// Used to quickly compare contact ids.
        /// </summary>
        [System.Runtime.InteropServices.FieldOffset(0)]
        public UInt32 Key;
    }

    /// <summary>
	/// A manifold point is a contact point belonging to a contact
	/// manifold. It holds details related to the geometry and dynamics
	/// of the contact points.
	/// The local point usage depends on the manifold type:
	/// -Circles: the local center of circleB
	/// -FaceA: the local center of cirlceB or the clip point of polygonB
	/// -FaceB: the clip point of polygonA
	/// This structure is stored across time steps, so we keep it small.
	/// Note: the impulses are used for internal caching and may not
	/// provide reliable contact forces, especially for high speed collisions.
	/// </summary>
    public class ManifoldPoint
    {
        /// <summary>
        /// Usage depends on manifold type.
        /// </summary
        public Vec2 LocalPoint;


        /// <summary>
        /// The non-penetration impulse.
        /// </summary>
        public float NormalImpulse;


        /// <summary>
        /// The friction impulse.
        /// </summary>
        public float TangentImpulse;


        /// <summary>
        /// Uniquely identifies a contact point between two shapes.
        /// </summary>
        public ContactID ID;

        public ManifoldPoint Clone ()
        {
            ManifoldPoint newPoint = new ManifoldPoint();
            newPoint.LocalPoint = this.LocalPoint;
            newPoint.NormalImpulse = this.NormalImpulse;
            newPoint.TangentImpulse = this.TangentImpulse;
            newPoint.ID = this.ID;
            return newPoint;
        }

    }


    public enum ManifoldType
    { 
      Circles, 
      FaceA,
      FaceB
    }

    /// <summary>
    /// A manifold for two touching convex shapes.
    /// </summary>
    public class Manifold
    {
        /// <summary>
        /// The points of contact.
        /// </summary>
        public ManifoldPoint[] points = new ManifoldPoint[Settings.MaxManifoldPoints];

        public Vec2 LocalPlanetNormal;

        /// <summary>
        /// Usage depends on manifold type.
        /// </summary>
        public Vec2 LocalPoint;


        public ManifoldType Type;

        /// <summary>
        /// The number of manifold points.
        /// </summary>
        public int PointCount;


        public Manifold()
        {
            for (int i = 0; i < Settings.MaxManifoldPoints; i++)
                Points[i] = new ManifoldPoint();

        }

        public Manifold Clone()
        {
            Manifold newManifold = new Manifold();
            newManifold.LocalPlanetNormal = this.LocalPlanetNormal;
            newManifold.LocalPoint = this.LocalPoint;
            newManifold.Type = this.Type;
            newManifold.PointCount = this.PointCount;
            int pointCount = this.Points.Length;
            ManifoldPoint[] tmp = new ManifoldPoint[pointCount];
            for (int i = 0; i < pointCount; i++)
            {
                tmp[i] = this.Points[i].Clone();
            }

            newManifold.points = tmp;
            return newManifold;
        }

    }


    /// <summary>
    /// A line segment.
    /// </summary>
    public struct Segment
    {
        // Collision Detection in Interactive 3D Environments by Gino van den Bergen
        // From Section 3.4.1
        // x = mu1 * p1 + mu2 * p2
        // mu1 + mu2 = 1 && mu1 >= 0 && mu2 >= 0
        // mu1 = 1 - mu2;
        // x = (1 - mu2) * p1 + mu2 * p2
        //   = p1 + mu2 * (p2 - p1)
        // x = s + a * r (s := start, r := end - start)
        // s + a * r = p1 + mu2 * d (d := p2 - p1)
        // -a * r + mu2 * d = b (b := s - p1)
        // [-r d] * [a; mu2] = b
        // Cramer's rule:
        // denom = det[-r d]
        // a = det[b d] / denom
        // mu2 = det[-r b] / denom
        /// <summary>
        /// Ray cast against this segment with another segment.        
        /// </summary>
        public bool TestSegment(out float lambda, out Vec2 normal, Segment segment, float maxLambda)
        {
            lambda = 0f;
            normal = new Vec2();

            Vec2 s = segment.P1;
            Vec2 r = segment.P2 - s;
            Vec2 d = P2 - P1;
            Vec2 n = Vec2.Cross(d, 1.0f);

            float k_slop = 100.0f * Common.Settings.FLT_EPSILON;
            float denom = -Vec2.Dot(r, n);

            //  Cull back  facing  collision and ignore  parallel  segments. 
            if (denom > k_slop)
            {
                // Does  the segment intersect the infinite  line  associated  with this segment? 
                Vec2 b = s - P1;
                float a = Vec2.Dot(b, n);

                if (0.0f <= a && a <= maxLambda *  denom)
                {
                    float mu2 = -r.X * b.Y + r.Y * b.X;

                    // Does  the segment  intersect  this  segment ? 
                    if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop))
                    {
                        a /= denom;
                        n.Normalize();
                        lambda = a;
                        normal = n;
                        return true;

                    }
                }

            }

            return false;
        }

        /// <summary>
		/// The starting point.
		/// </summary>
		public Vec2 P1;

        /// <summary>
        /// The ending point.
        /// </summary>
        public Vec2 P2;

    }


    /// <summary>
    /// An axis aligned bounding box.
    /// </summary>
    public struct AABB
    {
        /// <summary>
        /// The lower vertex.
        /// </summary>
        public Vec2 LowerBound;

        /// <summary>
        /// The upper vertex.
        /// </summary>
        public Vec2 UpperBound;


        /// Verify that the bounds are sorted.
        public bool IsValid
        {
          get
            {
                Vec2 d = UpperBound - LowerBound; 
                bool valid = d.X >= 0.0f && d.Y >= 0.0f;
                valid = valid && LowerBound.IsValid && UpperBound.IsValid;
                return valid;
            }
        }

        /// Get the center of the AABB.
        public  Vec2 Center
        {
            get { return 0.5f * (LowerBound + UpperBound); }
        }

        /// Combine two  extents  of the AABB (half-widths)
        public Vec2 Extents
        {
            get { return 0.5f * (UpperBound - LowerBound); }
        }

        /// Combine  two  AABBs  into this  one. 
        public void Combine(AABB aabb1, AABB  aabb2)
        {
            LowerBound = Common.Math.Min(aabb1.LowerBound, aabb2.LowerBound);
            UpperBound = Common.Math.Max(aabb1.UpperBound, aabb2.UpperBound);
        }

        // Does  this AABB  contain  the provided AABB.
        public  bool Contains(AABB aabb)
        {
            bool result = LowerBound.X <= aabb.LowerBound.X;
            result = result && LowerBound.Y <= aabb.LowerBound.Y;
            result = result && aabb.UpperBound.X <= UpperBound.X;
            result = result && aabb.UpperBound.X <= UpperBound.Y;
            return result;
        }


        /// <summary>
        // From Real-time Collision Detection, p179.
        /// </summary>
        
    }



}
