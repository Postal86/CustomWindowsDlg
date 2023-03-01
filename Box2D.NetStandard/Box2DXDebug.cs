using System.Diagnostics;


namespace Box2DX
{
    public static class Box2DXDebug
    {
        [Conditional("DEBUG")]
        public static void Assert(bool condition)
        {
            if (!condition)
            {
                condition = condition;
            }

            Debug.Assert(condition);
        }

        [Conditional("DEBUG")]
        public  static void Assert(bool condition, string message)
        {
            if (!condition)
            {
                condition = condition;
            }
            Debug.Assert(condition, message);
        }

        [Conditional("DEBUG")]
        public static void Assert(bool condition, string message, string detailMessage)
        {
            if (!condition)
            {
                condition = condition;
            }
            Debug.Assert(condition, message, detailMessage);
        }

        public static void ThrowBox2DXException(String message)
        {
            string msg = String.Format("Error: {0}", message);
            throw new Exception(msg);
        }

    }
}
