using RVO;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class ShapeObstacleBase : MonoBehaviour
{
    protected List<Vector2> mCacheVertexList = new List<Vector2>();
#if UNITY_EDITOR
    protected List<Vector3> mEditorDebugVertexList = new List<Vector3>();
    private void OnDrawGizmos()
    {
        if (mCacheVertexList != null && mEditorDebugVertexList.Count > 2)
        {
            Gizmos.color = Color.red;
            for (int i = 0; i < mCacheVertexList.Count - 1; ++i)
            {
                Gizmos.DrawLine(mEditorDebugVertexList[i], mEditorDebugVertexList[i + 1]);
            }
            int vCount = mEditorDebugVertexList.Count;
            Gizmos.DrawLine(mEditorDebugVertexList[vCount - 1], mEditorDebugVertexList[0]);

        }
    }
    private void OnValidate()
    {
        GenerateVertex();
    }
#endif
    internal void GenerateVertex(bool autoAddRvoSystem = false)
    {
        mCacheVertexList.Clear();
#if UNITY_EDITOR
        mEditorDebugVertexList.Clear();
#endif

        CalculatePoints();
        if (autoAddRvoSystem && mCacheVertexList.Count > 1)
        {
            AddObstacles();
        }
    }

    protected virtual void CalculatePoints()
    {
        
    }

    protected virtual void AddObstacles()
    {
        int obsId = Simulator.Instance.addObstacle(mCacheVertexList);
        Simulator.Instance.processObstacles();
    }
}
