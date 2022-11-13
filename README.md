# wrench_estimator_rtc
指定した部位でロボットが受けているwrenchを電流から推定する

サンプルは wrench_estimator_rtc/launch 以下

オフセットを取り除く場合は
```
rosservice call /WrenchEstimatorServiceROSBridge/removeWrenchEstimatorOffset 1
```
慣性の影響は未対応なので、静止しているときのみ対応。
