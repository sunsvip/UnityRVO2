using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RVO;
#if UNITY_EDITOR
using UnityEditor;

#endif
public class CircleObstacle : ShapeObstacleBase
{
    [SerializeField] float radius = 0.5f;
    [SerializeField][Range(3, 100)] int vertexCount = 6;
    protected override void CalculatePoints()
    {
        float anglePadding = 360f / vertexCount;

        var vec = transform.forward * radius;

        var curPos = transform.position;
        for (int i = 0; i < vertexCount; i++)
        {
            mCacheVertexList.Add(new Vector2(vec.x + curPos.x, vec.z + curPos.z));
#if UNITY_EDITOR
            mEditorDebugVertexList.Add(vec + curPos);
#endif
            vec = Quaternion.AngleAxis(-anglePadding, transform.up) * vec;
        }
    }
}
