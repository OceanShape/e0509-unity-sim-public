// 단계 7 — M2 상태 회신: /dsr01/joint_states 발행 (CONTRACT §3-1·§3-6, 결정 3).
//
// Isaac 이 하던 일을 그대로 한다.
//  - 물리 스텝마다(FixedUpdate, 60 Hz) 발행. 명령이 없어도(유휴) 계속 낸다.
//  - name = Isaac PhysX DOF 순서 10개 고정 배열. 가상 제어기가 앞 6칸을 **순서로** 읽기 때문이다(CONTRACT §2-2).
//  - position = ArticulationBody 가 실제로 움직인 jointPosition(측정값, rad). 명령값을 되돌려 보내지 않는다(D-4).
//  - 감싸지 않은 연속 각. 측정값은 ±180° 에서 감싸지 않는 것을 단계 5에서 확인했지만, 혹시 한 스텝에 180° 넘게 뛰면 펼친다.
//  - velocity·effort 는 빈 배열, stamp 는 벽시계, frame_id 는 빈 문자열.
// Mirror 모드(M1)에서는 발행하지 않는다. 같은 토픽을 구독하고 있어 자기 출력을 다시 받게 된다.
using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class JointStatePublisher : MonoBehaviour
{
    public JointStateMirror mirror;
    public string topic = "/dsr01/joint_states";
    [Tooltip("Isaac /dsr01/joint_states 의 name 순서 (CONTRACT §2-2). 바꾸지 말 것")]
    public string[] isaacOrder = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "rh_l1", "rh_p12_rn", "rh_l2", "rh_r2" };

    public long PublishCount { get; private set; }
    public float PublishHz { get; private set; }
    public bool Active { get; private set; }
    /// 마지막으로 낸 위치(rad, isaacOrder 순서). 테스트·HUD 용
    public double[] LastPositions => m_Positions;

    ROSConnection m_Ros;
    ArticulationBody[] m_Bodies;
    double[] m_Positions;
    float[] m_LastRaw;
    float[] m_Offset;
    bool m_HasLast;
    readonly Queue<double> m_Times = new Queue<double>();

    void Start()
    {
        if (mirror == null) mirror = GetComponent<JointStateMirror>();
        if (mirror == null) mirror = FindObjectOfType<JointStateMirror>();
        if (mirror == null) { Debug.LogError("[JointStatePublisher] JointStateMirror 가 없다"); enabled = false; return; }
        if (mirror.mode != JointStateMirror.ApplyMode.Command)
        {
            Debug.Log($"[JointStatePublisher] mode={mirror.mode} — 발행 안 함 (Command 모드에서만)");
            enabled = false;
            return;
        }

        m_Bodies = new ArticulationBody[isaacOrder.Length];
        for (int i = 0; i < isaacOrder.Length; i++)
        {
            if (!mirror.Joints.TryGetValue(isaacOrder[i], out m_Bodies[i]))
            {
                Debug.LogError($"[JointStatePublisher] 관절 '{isaacOrder[i]}' 이 로봇에 없다 — 발행 중단");
                enabled = false;
                return;
            }
        }
        m_Positions = new double[isaacOrder.Length];
        m_LastRaw = new float[isaacOrder.Length];
        m_Offset = new float[isaacOrder.Length];

        if (!JointStateMirror.SkipRos)
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterPublisher<JointStateMsg>(topic, 10);
        }
        Active = true;
        Debug.Log($"[JointStatePublisher] {topic} 발행 시작 (물리 스텝마다, Isaac 순서: {string.Join(",", isaacOrder)})");
    }

    void FixedUpdate()
    {
        if (!Active) return;
        for (int i = 0; i < m_Bodies.Length; i++)
        {
            float raw = m_Bodies[i].jointPosition[0];
            if (m_HasLast)
            {
                float jump = raw - m_LastRaw[i];
                if (jump > Mathf.PI) m_Offset[i] -= 2f * Mathf.PI;
                else if (jump < -Mathf.PI) m_Offset[i] += 2f * Mathf.PI;
            }
            m_LastRaw[i] = raw;
            m_Positions[i] = raw + m_Offset[i];
        }
        m_HasLast = true;

        if (m_Ros != null)
        {
            long ns = (DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()) * 1000000L;
            var stamp = new TimeMsg((int)(ns / 1000000000L), (uint)(ns % 1000000000L));
            var msg = new JointStateMsg(new HeaderMsg(stamp, ""), (string[])isaacOrder.Clone(), (double[])m_Positions.Clone(), new double[0], new double[0]);
            m_Ros.Publish(topic, msg);
        }
        PublishCount++;
        double now = Time.realtimeSinceStartupAsDouble;
        m_Times.Enqueue(now);
        while (m_Times.Count > 0 && now - m_Times.Peek() > 1.0) m_Times.Dequeue();
        PublishHz = m_Times.Count;
    }
}
