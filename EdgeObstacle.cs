using RVO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EdgeObstacle : ShapeObstacleBase
{
    [SerializeField] bool closed = true;
    [SerializeField] List<Vector3> points;

    protected override void CalculatePoints()
    {
        if (points == null) return;
        var center = transform.position;
        var rotation = Quaternion.Euler(0f, transform.eulerAngles.y, 0f);
        var scale = transform.lossyScale;
        for (int i = 0; i < points.Count; i++)
        {
            var point = points[i];
            point.x *= scale.x;
            point.z *= scale.z;
            point = center + rotation * point;
            mCacheVertexList.Add(new Vector2(point.x, point.z));
#if UNITY_EDITOR
            mEditorDebugVertexList.Add(point);
#endif
        }

        if (closed && mCacheVertexList.Count > 2)
        {
            mCacheVertexList.Add(mCacheVertexList[0]);
#if UNITY_EDITOR
            mEditorDebugVertexList.Add(mEditorDebugVertexList[0]);
#endif
        }
    }

    protected override void AddObstacles()
    {
        for (int i = 0; i < mCacheVertexList.Count - 1; i++)
        {
            int obsId = Simulator.Instance.addObstacle(new List<Vector2>() { mCacheVertexList[i], mCacheVertexList[i + 1] });
        }
        Simulator.Instance.processObstacles();
    }
}
