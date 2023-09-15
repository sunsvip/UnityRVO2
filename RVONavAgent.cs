using GameFramework;
using RVO;
using System;
using UnityEngine;

/// <summary>
/// RVO2寻路对象
/// </summary>
public class RVONavAgent : IReference
{
    readonly int GROUND_LAYER = LayerMask.GetMask("Ground");
    const float MIN_FLOAT_CHECK = 0.01f;
    public int Id { get; private set; }

    private float m_AvoidanceWeight = 0.5f;
    /// <summary>
    /// 移动速度
    /// </summary>
    public Vector2 Velocity
    {
        get => Simulator.Instance.getAgentVelocity(Id);
    }
    private Vector2 m_MoveDirection;
    /// <summary>
    /// 移动方向
    /// </summary>
    public Vector2 MoveDirection
    {
        get => m_MoveDirection;
        set
        {
            m_MoveDirection = value;
            Simulator.Instance.setAgentPrefVelocity(Id, m_MoveDirection);
        }
    }
    private float m_Radius;
    /// <summary>
    /// 碰撞体半径
    /// </summary>
    public float Radius
    {
        get => m_Radius;
        set
        {
            m_Radius = value;
            Simulator.Instance.setAgentRadius(Id, m_Radius);
        }
    }
    private float m_MaxSpeed;
    /// <summary>
    /// 最大移动速度
    /// </summary>
    public float MaxSpeed
    {
        get => m_MaxSpeed;
        set
        {
            m_MaxSpeed = value;
            Simulator.Instance.setAgentMaxSpeed(Id, m_MaxSpeed / LevelEntity.m_SyncStepTime);
        }
    }
    public Vector2 Position
    {
        get => Simulator.Instance.getAgentPosition(Id);
    }
    public float AvoidanceWeight
    {
        get => Simulator.Instance.getAgentWeight(Id);
        set
        {
            Simulator.Instance.setAgentWeight(Id, Mathf.Clamp(value, 0.01f, 1f));
        }
    }
    public bool IsReached { get; private set; }

    private bool m_StopMoving;
    public bool StopMoving
    {
        get => m_StopMoving;
        set
        {
            if (m_StopMoving == value) return;
            m_StopMoving = value;
            if (m_StopMoving)
            {
                Simulator.Instance.setAgentWeight(Id, 0.01f);
            }
            else
            {
                Simulator.Instance.setAgentWeight(Id, m_AvoidanceWeight);
            }
        }
    }
    private Transform m_Transform;
    private Transform m_MoveTarget; //移动目标节点
    private Vector3 m_MoveTargetPos; //移动目标点
    private float m_RotateSmoothSpeed = 10f;
    private float m_MoveSmoothSpeed = 20f;

    private Vector3 mCurrentPosition;
    private Quaternion mCurrentRotation;
    public static RVONavAgent Acquire(Transform transform, float radius, float maxSpeed)
    {
        var inst = ReferencePool.Acquire<RVONavAgent>();
        inst.m_Transform = transform;
        inst.Id = Simulator.Instance.addAgent(new Vector2(transform.position.x, transform.position.z));
        inst.Radius = radius;
        inst.MaxSpeed = maxSpeed;
        inst.m_StopMoving = false;
        inst.mCurrentPosition = transform.position;
        inst.mCurrentRotation = transform.rotation;
        GF.Event.Fire(null, ReferencePool.Acquire<RVONavAgentEventArgs>().Fill(inst));
        return inst;
    }
    /// <summary>
    /// 设置Agent移动和转向平滑度
    /// </summary>
    /// <param name="moveSmooth"></param>
    /// <param name="rotateSmooth"></param>
    public void SetAgentActionSmooth(float moveSmooth, float rotateSmooth)
    {
        this.m_MoveSmoothSpeed = moveSmooth;
        this.m_RotateSmoothSpeed = rotateSmooth;
    }
    public void LogicUpdate(float elapseSeconds)
    {
        if (m_MoveTarget == null) return;

        m_MoveTargetPos = m_MoveTarget.position;
        m_Transform.position = mCurrentPosition;
        m_Transform.rotation = mCurrentRotation;
    }
    /// <summary>
    /// 此方法由子线程调用,计算当前位置和方向
    /// </summary>
    /// <param name="elapseSeconds"></param>
    public void UpdatePositionAndRotation(float elapseSeconds)
    {
        var rvoPos = this.Position;
        var nextPos = mCurrentPosition;
        nextPos.x = rvoPos.x;
        nextPos.z = rvoPos.y;
        mCurrentPosition = Vector3.Lerp(mCurrentPosition, nextPos, elapseSeconds * m_MoveSmoothSpeed);
        if (StopMoving)
        {
            MoveDirection = Vector2.zero;
            return;
        }

        var offset = m_MoveTargetPos - mCurrentPosition;
        offset.y = 0;

        rvoPos.x = offset.x;
        rvoPos.y = offset.z;
        this.MoveDirection = RVOMath.normalize(rvoPos);
        var rvoForward = this.Velocity;
        if (RVOMath.absSq(rvoForward) > MIN_FLOAT_CHECK)
        {
            rvoForward = RVOMath.normalize(rvoForward);
            nextPos.Set(rvoForward.x, 0, rvoForward.y);
        }
        else
        {
            offset.y = 0;
            nextPos = offset.normalized;
        }
        mCurrentRotation = Quaternion.Lerp(mCurrentRotation, Quaternion.LookRotation(nextPos), elapseSeconds * m_RotateSmoothSpeed);
    }
    public void Clear()
    {
        Simulator.Instance.removeAgent(Id);
        Id = -1;
        m_Radius = 1;
        m_MaxSpeed = 0;
    }

    internal void SetMoveTarget(Transform transform)
    {
        m_MoveTarget = transform;
        m_MoveTargetPos = transform.position;
    }
    internal void SetMoveTarget(Vector3 point)
    {
        m_MoveTarget = null;
        m_MoveTargetPos = point;
    }
}
