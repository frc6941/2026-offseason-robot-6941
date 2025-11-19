# 硬件抽象模式 (Hardware Abstraction Pattern)

## 概述

这个项目使用**硬件抽象**来支持三种运行模式：
- 🤖 **真实机器人** (Real Robot) - 使用真实的硬件（TalonFX, CANcoder等）
- 🖥️ **模拟模式** (Simulation) - 使用物理模拟
- 📼 **日志回放** (Replay) - 从AdvantageKit日志回放数据

## 核心思想

每个subsystem使用**IO接口**来与硬件交互，而不是直接调用硬件API。根据运行模式，我们创建不同的IO实现：

```
┌─────────────────┐
│   Subsystem     │
└────────┬────────┘
         │ 使用
         ▼
┌─────────────────┐
│   MotorIO       │ ← 接口
└────────┬────────┘
         │ 实现
    ┌────┴─────┬─────────┐
    ▼          ▼         ▼
┌────────┐ ┌───────┐ ┌─────┐
│TalonFX │ │  Sim  │ │Empty│
│  IO    │ │  IO   │ │ IO  │
└────────┘ └───────┘ └─────┘
 真实硬件   物理模拟   日志回放
```

## 实现示例

### IntakePivotSubsystem

```java
public class IntakePivotSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public IntakePivotSubsystem() {
    super(
      IntakePivotConfig.CONFIG,
      new MotorInputsAutoLogged(),
      createIO(),  // ← 动态创建IO
      IntakePivotParamsNT.asServoMotorParamSources()
    );
  }

  private static MotorIO createIO() {
    if (Logger.hasReplaySource()) {
      // Replay模式：使用空IO，所有数据来自日志
      return new MotorIO() {};
    } else if (RobotBase.isReal()) {
      // 真实机器人：使用TalonFX硬件
      return new MotorIOTalonFX(IntakePivotConfig.CONFIG);
    } else {
      // 模拟模式：使用物理模拟
      return new MotorIOSim(IntakePivotConfig.SIM_CONFIG);
    }
  }
}
```

## 关键API

### 检测运行模式

```java
// 检测是否在真实机器人上运行
RobotBase.isReal()      // true = 真实硬件, false = 模拟

// 检测是否在回放AdvantageKit日志
Logger.hasReplaySource()  // true = 正在回放日志
```

### 运行模式优先级

检查顺序很重要：
1. **首先检查** `Logger.hasReplaySource()` - 最高优先级
2. **然后检查** `RobotBase.isReal()` - 区分真实vs模拟
3. **默认** - 模拟模式

## 为其他Subsystems应用此模式

### 模板代码

```java
public class YourSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public YourSubsystem() {
    super(
      YourConfig.CONFIG,
      new MotorInputsAutoLogged(),
      createIO(),
      YourParamsNT.asServoMotorParamSources()
    );
  }

  private static MotorIO createIO() {
    if (Logger.hasReplaySource()) {
      return new MotorIO() {};
    } else if (RobotBase.isReal()) {
      System.out.println("[YourSubsystem] Using REAL hardware");
      return new MotorIOTalonFX(YourConfig.CONFIG);
    } else {
      System.out.println("[YourSubsystem] Using SIMULATION");
      return new MotorIOSim(YourConfig.SIM_CONFIG);
    }
  }
}
```

## 优势

✅ **单一代码库** - 一份代码同时支持真实硬件和模拟  
✅ **自动切换** - 根据运行环境自动选择正确的IO  
✅ **易于测试** - 在电脑上模拟测试，然后直接部署到机器人  
✅ **日志回放** - 使用AdvantageKit回放比赛数据进行调试  
✅ **类型安全** - 使用泛型参数保证编译时类型检查  

## 注意事项

⚠️ **泛型类型要使用接口** - Subsystem的泛型参数应该是`MotorIO`接口，不是具体实现  
⚠️ **Config分离** - 真实硬件config和模拟config要分开（CONFIG vs SIM_CONFIG）  
⚠️ **调试输出** - 添加System.out.println来确认使用了正确的IO  

## 实际效果

### 部署到真实机器人时：
```
[IntakePivot] Using REAL hardware (TalonFX ID: 33)
[MotorIOTalonFX] ID: 33 | Setting MotionMagic: ...
```

### 运行模拟时：
```
[IntakePivot] Using SIMULATION
```

### 回放日志时：
```
[IntakePivot] Using empty IO (replay mode)
```

## 相关文件

- `lib/ironpulse/io/MotorIO.java` - IO接口定义
- `lib/ironpulse/io/MotorIOTalonFX.java` - 真实硬件实现
- `lib/ironpulse/io/MotorIOSim.java` - 模拟实现
- `lib/ironpulse/subsystem/ServoMotorSubsystem.java` - 基础subsystem类
