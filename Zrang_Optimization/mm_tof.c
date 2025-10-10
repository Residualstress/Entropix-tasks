#include "mm_tof.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "param.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (0.01745329251994329577f)
#endif

static float predictedTerrainHeight = 0.0f;
static float predictedElevation     = 0.0f;
static float measuredElevation      = 0.0f;

// 与 PR 逻辑配套的阈值（保持状态）
static float terrainThreshold = INFINITY;

// 开关参数（1=启用 PR 的“地形跳变处理”，0=回退为老逻辑）
static uint8_t zrangeOpt = 1;

PARAM_GROUP_START(mmtof)
  PARAM_ADD(PARAM_UINT8, ZrangeOpt, &zrangeOpt)
PARAM_GROUP_STOP(mmtof)

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = (arm_matrix_instance_f32){1, KC_STATE_DIM, h};

  // 仅当 R[2][2] 合理时更新（与 PR 前置条件一致）
  if (fabsf(this->R[2][2]) > 0.1f && this->R[2][2] > 0.0f) {
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }

    float cosA = cosf(angle);
    if (cosA < 1e-3f) cosA = 1e-3f;               // 数值保护

    float predictedDistance = this->S[KC_STATE_Z] / cosA;
    predictedElevation      = this->S[KC_STATE_Z];

    float measuredDistance  = tof->distance;      // [m]
    measuredElevation       = tof->distance * cosA + predictedTerrainHeight; // [m]

    // 观测矩阵：Z 上的增益
    h[KC_STATE_Z] = 1.0f / cosA;

    float residual;

    if (zrangeOpt) {
      // —— 与 PR #1431 对齐的地形跳变处理 ——  :contentReference[oaicite:1]{index=1}
      residual = measuredDistance - predictedDistance;

      // 若预测/测得的“海拔高度”差超过阈值，则认为地形发生跳变，调整地形高度
      if (fabsf(predictedElevation - measuredElevation) > terrainThreshold) {
        predictedTerrainHeight += (predictedElevation - measuredElevation);
      }

      // 首轮后设置阈值（PR 注释：25 Hz 更新、最大 1 m/s，约 4 cm 可视为正常变化）
      terrainThreshold = 0.04f;

      // 残差加入地形高度补偿
      residual += predictedTerrainHeight;

      // 单次标量更新（PR 版本）
      kalmanCoreScalarUpdate(this, &H, residual, tof->stdDev);

    } else {
      // —— 关闭优化：回退为老逻辑（无地形跳变处理） ——
      residual = measuredDistance - predictedDistance;
      kalmanCoreScalarUpdate(this, &H, residual, tof->stdDev);
    }
  }
}

LOG_GROUP_START(mmtof)
  LOG_ADD(LOG_FLOAT, prdTerrainHeight, &predictedTerrainHeight)
  LOG_ADD(LOG_FLOAT, prdElevation, &predictedElevation)
  LOG_ADD(LOG_FLOAT, measElevation, &measuredElevation)
LOG_GROUP_STOP(mmtof)
