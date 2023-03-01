/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/

// Notes:
// - we use bound arrays instead of linked lists for cache coherence.
// - we use quantized integral values for fast compares.
// - we use short indices rather than pointers to save memory.
// - we use a stabbing count for fast overlap queries (less than order N).
// - we also use a time stamp on each proxy to speed up the registration of
//   overlap query results.
// - where possible, we compare bound indices instead of values to reduce
//   cache misses (TODO_ERIN).
// - no broadphase is perfect and neither is this one: it is not great for huge
//   worlds (use a multi-SAP instead), it is not great for large objects.


#define ALLOWUNSAFE

using Box2D.Common;
using Box2DX;

namespace Box2D.NetStandard.Collision
{

    public delegate float SortKeyFunc(object shape);

#warning "CAS"
    public class BroadValues
    {
        public ushort[/*2*/] LowerValues = new ushort[2];
        public ushort[/*2*/] UpperValues = new ushort[2];
    }
#warning "CAS"
    public class Bound
    {
        public bool IsLower { get { return (Value & (ushort)1) == (ushort)0; } }
        public bool IsUpper { get { return (Value & (ushort)1) == (ushort)1; } }

        public ushort Value;
        public ushort ProxyId;
        public ushort StabbingCount;

        public Bound Clone()
        {
            Bound newBound = new Bound();
            newBound.Value = this.Value;
            newBound.ProxyId = this.ProxyId;
            newBound.StabbingCount = this.StabbingCount;
            return newBound;
        }
    }
#warning "CAS"
    public class Proxy
    {
        public ushort[/*2*/] LowerBounds = new ushort[2], UpperBounds = new ushort[2];
        public ushort OverlapCount;
        public ushort TimeStamp;
        public object UserData;

        public ushort Next
        {
            get { return LowerBounds[0]; }
            set { LowerBounds[0] = value; }
        }

        public bool IsValid { get { return OverlapCount != BroadPhase.Invalid; } }


    }

    public class BroadPhase
    {
#if TARGET_FLOAT32_IS_FIXED
    public static readonly ushort BROADPHASE_MAX = (Common.Math.USHRT_MAX/2);
#else
        public static readonly ushort BROADPHASE_MAX = Common.Math.USHRT_MAX;
#endif

        public static readonly ushort Invalid = BROADPHASE_MAX;
        public static readonly ushort NullEdge = BROADPHASE_MAX;

        public PairManager _pairManager;

        public Proxy[] _proxyPool = new Proxy[Settings.MaxProxies];
        public ushort _freeProxy;

        public Bound[][] _bounds = new Bound[2][/*(2 * Settings.MaxProxies)*/];

        public ushort[] _queryResults = new ushort[Settings.MaxProxies];
        public float[] _querySortKeys = new float[Settings.MaxProxies];
        public int _queryResultCount;

        public AABB _worldAABB;
        public Vec2 _quantizationFactor;
        public int _proxyCount;
        public ushort _timeStamp;

        public static bool IsValidate = false;

        public BroadPhase(AABB worldAABB,  PairCallback callback)
        {
            _pairManager = new PairManager();
            _pairManager.Initialize(this, callback);

            Box2DXDebug.Assert(_worldAABB.IsValid);
            _worldAABB = worldAABB;
            _proxyCount = 0;

            Vec2 d = _worldAABB.UpperBound - worldAABB.LowerBound;
            _quantizationFactor.X = (float)BROADPHASE_MAX / d.X;
            _quantizationFactor.Y = (float)BROADPHASE_MAX / d.Y;

            for (ushort i = 0; i <  Settings.MaxProxies - 1; ++i)
            {
                _proxyPool[i] = new Proxy();
                _proxyPool[i].Next = (ushort)(i + 1);
                _proxyPool[i].TimeStamp = 0;
                _proxyPool[i].OverlapCount = BroadPhase.Invalid;
                _proxyPool[i].UserData = null;
            }

            _proxyPool[Settings.MaxProxies - 1] = new Proxy();
            _proxyPool[Settings.MaxProxies - 1].Next = PairManager.NullProxy;
            _proxyPool[Settings.MaxProxies - 1].TimeStamp = 0;
            _proxyPool[Settings.MaxProxies - 1].OverlapCount = BroadPhase.Invalid;
            _proxyPool[Settings.MaxProxies - 1].UserData = null;
            _freeProxy = 0;

            _timeStamp = 1;
            _queryResultCount = 0;

            for (int i = 0; i < 2; i++)
            {
                _bounds[i] = new Bound[(2 * Settings.MaxProxies)];
            }

            int bCount = 2 * Settings.MaxProxies;
            for (int j = 0; j < 2; j++)
                for (int k = 0; k < bCount; k++)
                    _bounds[j][k] = new Bound();
            
            
        }

        // Use this to see if your proxy is in range. If it is not in range,
        // it should be destroyed. Otherwise you may get O(m^2) pairs, where m
        // is the number of proxies that are out of range.
        public bool InRange(AABB  aabb)
        {
            Vec2 d = Common.Math.Max(aabb.LowerBound - _worldAABB.UpperBound, _worldAABB.LowerBound - aabb.UpperBound);
            return Common.Math.Max(d.X, d.Y) < 0.0f;
        }

        // Create and destroy proxies. These call Flush first.
        public ushort  CreateProxy(AABB aabb, object  userData)
        {
            Box2DXDebug.Assert(_proxyCount < Settings.MaxProxies);
            Box2DXDebug.Assert(_freeProxy != PairManager.NullProxy);

            ushort proxyId = _freeProxy;
            Proxy proxy = _proxyPool[proxyId];
            _freeProxy = proxy.Next;

            proxy.OverlapCount = 0;
            proxy.UserData = userData;

            int boundCount = 2 * _proxyCount;

            ushort[] lowerValues = new ushort[2], upperValus = new ushort[2];
            ComputeBounds(out lowerValues, out upperValus, aabb);

            for (int axis = 0; axis < 2; ++axis)
            {
                Bound[] bounds = _bounds[axis];
                int lowerIndex, upperIndex;
                Query(out lowerIndex, out upperIndex, lowerValues[axis], bounds, boundCount, axis);

#warning "Check this" 
                // memmove(bounds + upperIndex + 2, bounds + upperIndex, (boundCount - upperIndex) * sizeof(b2Bound));
                Bound[] tmp = new Bound[boundCount - upperIndex];
                for(int i = 0; i < (boundCount - upperIndex); i++)
                {
                    tmp[i] = bounds[upperIndex + i].Clone();
                }
                for (int i = 0; i < (boundCount - upperIndex); i++)
                {
                    bounds[upperIndex + 2 + i] = tmp[i];
                }

                //  memmove(bounds + lowerIndex + 1, bounds + lowerIndex, (upperIndex - lowerIndex) * sizeof(b2Bound));
                tmp = new Bound[upperIndex - lowerIndex];
                for (int i = 0; i < (upperIndex - lowerIndex); i++)
                {
                    tmp[i] = bounds[lowerIndex + i].Clone();
                }
                for (int  i = 0; i < (upperIndex - lowerIndex); i++)
                {
                    bounds[lowerIndex + 1 + i] = tmp[i];
                }

                // The upper index has  increased because of the  lower bound  insertion
                ++upperIndex;

                // Copy in the  new bounds
                bounds[lowerIndex].Value = lowerValues[axis];
                bounds[lowerIndex].ProxyId = proxyId;
                bounds[upperIndex].Value = upperValus[axis];
                bounds[upperIndex].ProxyId = proxyId;

                bounds[lowerIndex].StabbingCount = lowerIndex == 0 ? (ushort)0 : bounds[lowerIndex - 1].StabbingCount;
                bounds[upperIndex].StabbingCount = bounds[upperIndex - 1].StabbingCount;

                //  Adjust  the  stabbing  count between  the new bounds.
                for ()
            }
        }
    }
}
