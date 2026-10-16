/**
 * PC 验证程序：测试 GM6020 协议编解码、PID、正弦轨迹。
 * 只包含硬件无关的模块，不依赖 HAL。
 *
 * 编译： g++ -std=c++17 -I<inc> test_pc.cpp <src...> -o test_pc
 */

#include <cstdio>
#include <cmath>
#include <cstring>

#include "gm6020.hpp"
#include "pid.hpp"
#include "trajectory.hpp"

static int g_fail = 0;

static void Check(bool ok, const char *what)
{
  printf("  [%s] %s\n", ok ? "OK " : "FAIL", what);
  if (!ok)
  {
    g_fail++;
  }
}

/* 把 float 打包成 CAN 大端 2 字节 */
static void PutI16(uint8_t *p, int16_t v)
{
  p[0] = (uint8_t)((v >> 8) & 0xFF);
  p[1] = (uint8_t)(v & 0xFF);
}

/* ---------- 1. 标识符 ---------- */
static void TestIds()
{
  printf("\n== 1. CAN 标识符 ==\n");
  Gm6020 m1(1), m4(4), m5(5), m7(7);

  Check(m1.RxId() == 0x205, "ID1 反馈 ID = 0x205");
  Check(m7.RxId() == 0x20B, "ID7 反馈 ID = 0x20B");
  Check(m1.TxId() == 0x1FF, "ID1 控制 ID = 0x1FF");
  Check(m4.TxId() == 0x1FF, "ID4 控制 ID = 0x1FF");
  Check(m5.TxId() == 0x2FF, "ID5 控制 ID = 0x2FF");
}

/* ---------- 2. 控制帧编码位置 ---------- */
static void TestEncode()
{
  printf("\n== 2. 控制帧编码位置 ==\n");
  uint8_t buf[8];

  /* ID1 应写在 [0,1] */
  memset(buf, 0, 8);
  Gm6020 m1(1);
  m1.SetVoltage(1000);
  m1.Encode(buf);
  Check(buf[0] == 0x03 && buf[1] == 0xE8, "ID1 -> 字节[0,1] = 1000");
  Check(buf[2] == 0 && buf[7] == 0, "ID1 不动其他字节");

  /* 负电压 */
  memset(buf, 0, 8);
  m1.SetVoltage(-1000);
  m1.Encode(buf);
  Check(buf[0] == 0xFC && buf[1] == 0x18, "ID1 负电压 -1000 编码正确");

  /* ID3 应写在 [4,5] */
  memset(buf, 0, 8);
  Gm6020 m3(3);
  m3.SetVoltage(5000);
  m3.Encode(buf);
  Check(buf[4] == 0x13 && buf[5] == 0x88, "ID3 -> 字节[4,5] = 5000");

  /* ID5 应写在 [0,1]（0x2FF 帧） */
  memset(buf, 0, 8);
  Gm6020 m5(5);
  m5.SetVoltage(1234);
  m5.Encode(buf);
  Check(buf[0] == 0x04 && buf[1] == 0xD2, "ID5 -> 字节[0,1] = 1234");
}

/* ---------- 3. 反馈解析 + 多圈角度 ---------- */
static void TestDecode()
{
  printf("\n== 3. 反馈解析与多圈角度 ==\n");
  Gm6020 m(1);
  uint8_t d[8];

  /* 角度 0，转速 0 */
  PutI16(d + 0, 0);
  PutI16(d + 2, 0);
  PutI16(d + 4, 0);
  d[6] = 25; d[7] = 0;
  m.Decode(d);
  Check(std::fabs(m.Angle() - 0.0f) < 1e-4f, "角度 0 -> 0 rad");
  Check(std::fabs(m.Temp() - 25.0f) < 1e-4f, "温度 25 摄氏度");

  /* 角度 4096（半圈）-> pi rad */
  PutI16(d + 0, 4096);
  m.Decode(d);
  Check(std::fabs(m.Angle() - 3.14159f) < 1e-3f, "角度 4096 -> pi rad");

  /* 转速 60 rpm -> 1 圈/秒 = 2*pi rad/s */
  PutI16(d + 2, 60);
  m.Decode(d);
  Check(std::fabs(m.Vel() - 6.28319f) < 1e-3f, "转速 60 rpm -> 2*pi rad/s");

  /* 跨零：4096 之后到 8000，再回到 100，应累计到第二圈 */
  PutI16(d + 0, 4096); m.Decode(d);
  PutI16(d + 0, 8000); m.Decode(d);
  float before = m.Angle();
  PutI16(d + 0, 100);  m.Decode(d);          /* 8191->0 跨零，进第二圈 */
  float after = m.Angle();
  /* 期望 after 约为 (1 圈 + 100/8192 圈) = 2*pi*(1+100/8192) ≈ 6.36 */
  float expect = 2.0f * 3.14159265f * (1.0f + 100.0f / 8192.0f);
  Check(after > before, "跨零后角度继续增大（多圈连续）");
  Check(std::fabs(after - expect) < 0.05f, "跨零后角度值正确");
}

/* ---------- 4. PID ---------- */
static void TestPid()
{
  printf("\n== 4. PID ==\n");
  Pid p(1.0f, 0.0f, 0.0f, 100.0f, -100.0f);

  /* 纯 P：误差 10 -> 输出 10 */
  Check(std::fabs(p.Calc(10.0f, 0.0f) - 10.0f) < 1e-4f, "纯P：误差10 -> 输出10");

  /* 输出限幅 */
  Pid p2(100.0f, 0.0f, 0.0f, 100.0f, -100.0f);
  Check(std::fabs(p2.Calc(10.0f, 0.0f) - 100.0f) < 1e-4f, "限幅：饱和到上限100");
  Check(std::fabs(p2.Calc(-10.0f, 0.0f) + 100.0f) < 1e-4f, "限幅：饱和到下限-100");

  /* 积分：Ki=1000, T=0.001，误差恒 1，积分项 = 1000*0.001*n = n */
  Pid p3(0.0f, 1000.0f, 0.0f, 1e9f, -1e9f);
  for (int i = 0; i < 5; i++)
  {
    p3.Calc(1.0f, 0.0f);
  }
  float out5 = p3.Calc(1.0f, 0.0f);   /* 第 6 拍，积分累加 6 */
  Check(std::fabs(out5 - 6.0f) < 1e-3f, "纯I：第6拍输出 = 6（积分累加正确）");

  /* 抗积分饱和：持续饱和，撤出后应能较快回落 */
  Pid p4(0.0f, 1000.0f, 0.0f, 5.0f, -5.0f);
  for (int i = 0; i < 100; i++)
  {
    p4.Calc(100.0f, 0.0f);   /* 长时间正饱和 */
  }
  float back = p4.Calc(-1.0f, 0.0f);
  Check(back < 5.0f, "抗积分饱和：反向输入后输出离开上限");
}

/* ---------- 5. 正弦轨迹 ---------- */
static void TestTrajectory()
{
  printf("\n== 5. 正弦轨迹 ==\n");
  Trajectory t;

  t.Reset();
  Check(std::fabs(t.Time()) < 1e-6f, "复位后 t = 0");

  /* t=0 时 sin(0)=0 */
  t.Step();  /* 推进到第 1 拍，t=0.001 */
  float v1 = t.VelRef();
  float p1 = t.PosRef();
  printf("      第1拍: v=%.4f rad/s, pos=%.4f rad\n", v1, p1);

  /* 跑到 t = 1/(4f) 处，sin 应接近峰值 */
  float fv = 0.25f, fp = 0.20f;
  t.Reset();
  uint32_t n = (uint32_t)(0.25f / fv / 0.001f);   /* 到达速度峰值所需拍数 */
  for (uint32_t i = 0; i < n; i++)
  {
    t.Step();
  }
  float vPeak = t.VelRef();
  printf("      速度峰值处: v=%.4f rad/s (期望接近幅值)\n", vPeak);
  Check(vPeak > 0.0f, "速度曲线能到达正值");

  /* 幅值范围检查：跑一整圈速度周期，找最大最小值 */
  t.Reset();
  float vmax = -1e9f, vmin = 1e9f;
  uint32_t steps = (uint32_t)(1.0f / fv / 0.001f);
  for (uint32_t i = 0; i < steps; i++)
  {
    t.Step();
    float v = t.VelRef();
    if (v > vmax) vmax = v;
    if (v < vmin) vmin = v;
  }
  printf("      速度波形范围: [%.3f, %.3f]\n", vmin, vmax);
  Check(vmax > 5.0f && vmax < 7.0f, "速度幅值约 6 rad/s");
  Check(vmin < -5.0f && vmin > -7.0f, "速度负向幅值约 -6 rad/s");

  /* 位置曲线范围 */
  t.Reset();
  float pmax = -1e9f, pmin = 1e9f;
  steps = (uint32_t)(1.0f / fp / 0.001f);
  for (uint32_t i = 0; i < steps; i++)
  {
    t.Step();
    float p = t.PosRef();
    if (p > pmax) pmax = p;
    if (p < pmin) pmin = p;
  }
  printf("      位置波形范围: [%.3f, %.3f]\n", pmin, pmax);
  Check(pmax > 1.9f && pmax < 2.1f, "位置幅值约 2 rad");
  Check(pmin < -1.9f && pmin > -2.1f, "位置负向幅值约 -2 rad");
}

int main()
{
  printf("==== GM6020 考核代码 · PC 验证 ====\n");
  TestIds();
  TestEncode();
  TestDecode();
  TestPid();
  TestTrajectory();

  printf("\n==== 结果: %s（失败 %d 项）====\n",
         g_fail == 0 ? "全部通过" : "有失败", g_fail);
  return g_fail == 0 ? 0 : 1;
}
