using RVO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// BoxCollider转化为RVO障碍物
/// </summary>
public class BoxObstacle : ShapeObstacleBase
{
    [SerializeField] UnityEngine.Vector2 boxSize = UnityEngine.Vector2.one;
    [SerializeField] UnityEngine.Vector2 center = UnityEngine.Vector2.one;

    protected override void CalculatePoints()
    {
        var curPos = transform.position;
        var halfSize = new UnityEngine.Vector2(transform.lossyScale.x, transform.lossyScale.z) * boxSize * 0.5f;
        var rotation = Quaternion.Euler(0f, transform.eulerAngles.y, 0f);
        var pointA = curPos + rotation * (Vector3.right * (halfSize.x + center.x) + Vector3.forward * (halfSize.y + center.y));
        var pointB = curPos + rotation * (Vector3.left * (halfSize.x - center.x) + Vector3.forward * (halfSize.y + center.y));
        var pointC = curPos + rotation * (Vector3.left * (halfSize.x - center.x) + Vector3.back * (halfSize.y - center.y));
        var pointD = curPos + rotation * (Vector3.right * (halfSize.x + center.x) + Vector3.back * (halfSize.y - center.y));

        mCacheVertexList.Add(new Vector2(pointA.x, pointA.z));
        mCacheVertexList.Add(new Vector2(pointB.x, pointB.z));
        mCacheVertexList.Add(new Vector2(pointC.x, pointC.z));
        mCacheVertexList.Add(new Vector2(pointD.x, pointD.z));
#if UNITY_EDITOR
        mEditorDebugVertexList.Add(pointA);
        mEditorDebugVertexList.Add(pointB);
        mEditorDebugVertexList.Add(pointC);
        mEditorDebugVertexList.Add(pointD);
#endif
    }
}
