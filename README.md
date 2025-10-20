# 3D RVE Model Generator for ABAQUS

[English](#english) | [中文](#chinese)

<a name="english"></a>

## 📋 Overview

A powerful Python script for automated generation of **3D Representative Volume Element (RVE)** models in ABAQUS with controllable fiber volume fraction. This tool extends the 2D RVE generator to three dimensions, enabling realistic composite material micromechanics simulation with continuous cylindrical fibers.

### ✨ Key Features

- **3D Fiber Architecture**: Continuous cylindrical fibers aligned along Z-axis
- **Controllable Volume Fraction**: Automatic generation based on user-defined fiber volume fraction (Vf)
- **Advanced Placement Algorithm**: Hybrid algorithm (RSA seeding + anchored relaxation + forced correction) for optimal fiber distribution in XY plane
- **Orthotropic Fiber Material**: Realistic fiber properties with directional stiffness (E1, E2, G12, G23, ν12)
- **Minimum Distance Guarantee**: Ensures spacing between fibers (in XY plane) with error reporting
- **Complete Material System**: Automatic creation of fiber (orthotropic) + matrix (Drucker-Prager) + interface (cohesive)
- **3D Periodic Boundary Conditions**: XYZ direction periodicity for accurate RVE behavior
- **3D Cohesive Elements**: COH3D8 elements at fiber-matrix interfaces
- **Precise Phase Identification**: Geometric algorithm for accurate fiber and matrix volume recognition
- **Automatic Material Orientation**: Fiber material Direction-1 aligned with +Z axis
- **CSV Export**: Fiber center coordinates and geometry data
- **Distance Verification**: Validates inter-fiber spacing in XY plane with statistics

## 🔬 3D vs 2D Comparison

| Feature | 2D RVE | 3D RVE (This Script) |
|---------|--------|----------------------|
| **Fiber Geometry** | Circular cross-section | Continuous cylinder along Z-axis |
| **Fiber Material** | Isotropic | Orthotropic (realistic) |
| **PBCs** | XY directions | XYZ directions |
| **Cohesive Elements** | COH2D4 | COH3D8 |
| **Distance Check** | XY plane | XY plane only (Z continuous) |
| **Mesh Complexity** | Baseline | 10-50× larger |
| **Computation Cost** | Moderate | Significantly higher |
| **Realism** | Good for 2D analysis | Closer to real materials |

## 🎯 Applications

- Unidirectional composite material simulation
- Fiber-reinforced composite micromechanics
- Interfacial debonding and damage studies
- Effective property prediction
- Progressive damage analysis
- Multiscale modeling
- Composite manufacturing process simulation

## 🔧 Requirements

### Software Requirements
- **ABAQUS**: Version 2023 or compatible
- **Python**: 2.7 (ABAQUS built-in)

### Hardware Recommendations
- **RAM**: Minimum 16 GB (32 GB recommended for large models)
- **CPU**: Multi-core processor (8+ cores recommended)
- **Disk**: 5-10 GB free space per model
- **GPU**: Not required (ABAQUS is CPU-based)

### Python Modules
All required modules are included in ABAQUS installation.

## 📥 Installation

1. **Clone or Download**
   ```bash
   git clone https://github.com/yourusername/3D_RVE_Model.git
   cd 3D_RVE_Model
   ```

2. **Verify ABAQUS**
   ```bash
   abaqus information=system
   ```

## 🚀 Quick Start

### Basic Usage

1. **Edit Parameters** (Lines 1747-1960):
   ```python
   # Basic geometry
   RVE_SIZE = [1.0, 1.0, 2.0]     # [Width, Height, Depth] in mm
   FIBER_RADIUS = 0.05             # Fiber radius in mm
   TARGET_VF = 0.50                # Target volume fraction (0.0-1.0)
   
   # Mesh settings
   GLOBAL_SEED_SIZE = 0.02         # Global mesh seed size
   ```

2. **Run in ABAQUS**
   ```bash
   # Command line (no GUI)
   abaqus cae noGUI=3D_RVE_Model.py
   
   # Or in ABAQUS CAE GUI
   File → Run Script → Select 3D_RVE_Model.py
   ```

3. **Expected Time**
   - Simple model (Vf=0.3, small size): 5-15 minutes
   - Standard model (Vf=0.5, medium size): 15-45 minutes
   - Complex model (Vf=0.6, large size): 1-3 hours

## 📖 Parameter Guide

### Geometry Parameters

```python
# RVE_SIZE: [Width, Height, Depth] in mm
# - Width & Height: XY plane dimensions (fiber distribution area)
# - Depth: Z-axis length (fiber length)
# - Typical: [1.0, 1.0, 2.0] for aspect ratio testing
RVE_SIZE = [1.0, 1.0, 2.0]

# FIBER_RADIUS: Radius of cylindrical fibers in mm
# - Typical: 0.003-0.1 mm depending on fiber type
# - Carbon fiber: ~0.003-0.005 mm
# - Glass fiber: ~0.005-0.01 mm
FIBER_RADIUS = 0.05

# TARGET_VF: Target fiber volume fraction
# - Range: 0.0 to 1.0 (recommend 0.3-0.6)
# - Typical composites: 0.5-0.65
# - High performance: up to 0.7 (challenging)
TARGET_VF = 0.50

# MIN_DIST_FACTOR: Minimum distance factor
# - Multiplied by fiber diameter to get minimum spacing
# - Range: 0.001-0.05
# - Smaller: denser packing, harder to mesh
# - Larger: sparser distribution, easier to mesh
MIN_DIST_FACTOR = 0.01
```

### Mesh Parameters

```python
# GLOBAL_SEED_SIZE: Element size in mm
# - Critical for 3D models (impacts computation significantly)
# - Smaller: More accurate, much slower
# - Typical range: 0.01-0.05 mm
# - Recommendation: Start with 0.03, refine as needed
GLOBAL_SEED_SIZE = 0.02

# DEVIATION_FACTOR: Mesh deviation tolerance
# - Controls mesh quality on curved surfaces
# - Range: 0.05-0.2
# - Smaller: Better quality, more elements
DEVIATION_FACTOR = 0.1

# MIN_SIZE_FACTOR: Minimum element size factor
# - Prevents extremely small elements
# - Multiplied by global seed size
# - Range: 0.05-0.5
MIN_SIZE_FACTOR = 0.1

# PAIRING_TOLERANCE_FACTOR: Node pairing tolerance for PBCs
# - Multiplied by global seed size
# - Adjust if PBC pairing fails
# - Typical: 3.0-10.0
PAIRING_TOLERANCE_FACTOR = 5.0
```

### Fiber Distribution Control

```python
# RSA_SEEDING_RATIO: Controls fiber placement algorithm
# 
# HIGH VALUES (0.8-1.0) - "Fast Uniform Mode":
#   - Quick generation
#   - Uniform distribution
#   - Recommended for production
#   - Example: 0.9 = 90% RSA, 10% relaxation
#
# LOW VALUES (0.0-0.3) - "Physical Clustering Mode":
#   - Realistic clustering
#   - Slower generation
#   - Closer to real materials
#   - Example: 0.1 = 10% RSA, 90% relaxation
#
# MEDIUM VALUES (0.4-0.7) - "Balanced Mode":
#   - Mix of both approaches
#   - Moderate speed
#   - Example: 0.5 = 50% RSA, 50% relaxation

RSA_SEEDING_RATIO = 0.9
```

### Material Parameters

#### Fiber Material (Orthotropic)

```python
# Longitudinal properties (along fiber axis = Z-axis)
FIBER_E1 = 235.0      # Longitudinal modulus (GPa)
FIBER_NU12 = 0.2      # Major Poisson's ratio

# Transverse properties (perpendicular to fiber axis)
FIBER_E2 = 15.0       # Transverse modulus (GPa)

# Shear properties
FIBER_G12 = 15.0      # Longitudinal shear modulus (GPa)
FIBER_G23 = 7.0       # Transverse shear modulus (GPa)

# Typical values:
# - Carbon fiber: E1~235 GPa, E2~15 GPa, G12~15 GPa
# - Glass fiber: E1~72 GPa, E2~72 GPa, G12~30 GPa
```

#### Matrix Material (Drucker-Prager)

```python
MATRIX_E = 3170.0                      # Elastic modulus (MPa)
MATRIX_NU = 0.35                       # Poisson's ratio
MATRIX_FRICTION_ANGLE = 16.0           # Friction angle (degrees)
MATRIX_FLOW_STRESS_RATIO = 1.0         # Flow stress ratio
MATRIX_DILATION_ANGLE = 16.0           # Dilation angle (degrees)
MATRIX_HARDENING_YIELD = 106.4         # Yield stress (MPa)
MATRIX_HARDENING_PLASTIC_STRAIN = 0.0  # Plastic strain at yield
MATRIX_DAMAGE_STRAIN = 0.01            # Damage initiation strain
MATRIX_DAMAGE_DISPLACEMENT = 5e-05     # Damage displacement (mm)
```

#### Cohesive Interface Material

```python
# Stiffness (N/mm³)
COHESIVE_K_NN = 1e8    # Normal stiffness
COHESIVE_K_SS = 1e8    # First shear stiffness
COHESIVE_K_TT = 1e8    # Second shear stiffness

# Strength (MPa)
COHESIVE_T_N = 44.0    # Normal strength
COHESIVE_T_S = 82.0    # First shear strength
COHESIVE_T_T = 82.0    # Second shear strength

# Fracture energy (N/mm)
COHESIVE_GIC = 0.001   # Mode I
COHESIVE_GIIC = 0.002  # Mode II
COHESIVE_GIIIC = 0.002 # Mode III

# BK criterion
COHESIVE_ETA = 1.5     # Exponent (1.0-3.0)

# Numerical stability
COHESIVE_STAB_COEFF = 0.0001
```

## 📊 Output Files

### ABAQUS Model
- **Model Name**: `3D-RVE-Vf-XX` (XX = volume fraction percentage)
- **Contents**:
  - 3D cylindrical fiber geometry
  - Material definitions with orientation
  - Cohesive interface elements
  - 3D periodic boundary conditions (XYZ)
  - Meshed assembly ready for analysis

### CSV Export
- **Filename**: `FiberCenters_3D_VfXX_YYYYMMDD_HHMMSS.csv`
- **Format**:
  ```csv
  # 3D RVE Fiber Center Coordinates
  # RVE Size (Width x Height x Depth): 1.000000 x 1.000000 x 2.000000
  # Fiber Radius: 0.050000
  # Fiber Length: 2.000000
  # Target Volume Fraction: 0.5000
  # Total Fiber Count: 127
  Fiber_ID,X_Coordinate,Y_Coordinate,Z_Start,Z_End
  1,0.12345678,0.23456789,0.00000000,2.00000000
  2,0.34567890,0.45678901,0.00000000,2.00000000
  ...
  ```

## 💡 Usage Examples

### Example 1: Standard Unidirectional Composite

```python
RVE_SIZE = [1.0, 1.0, 2.0]
FIBER_RADIUS = 0.05
TARGET_VF = 0.50
GLOBAL_SEED_SIZE = 0.02
RSA_SEEDING_RATIO = 0.9

# Carbon fiber properties
FIBER_E1 = 235.0
FIBER_E2 = 15.0
FIBER_G12 = 15.0
FIBER_G23 = 7.0
FIBER_NU12 = 0.2

# Epoxy matrix
MATRIX_E = 3500.0
MATRIX_NU = 0.35
```
**Time**: ~20 minutes  
**Use**: General composite testing

### Example 2: High Fiber Content

```python
RVE_SIZE = [1.0, 1.0, 1.5]
FIBER_RADIUS = 0.04
TARGET_VF = 0.65
GLOBAL_SEED_SIZE = 0.025
MIN_DIST_FACTOR = 0.005
RSA_SEEDING_RATIO = 0.7
```
**Time**: ~45 minutes  
**Use**: High-performance composites

### Example 3: Realistic Clustering

```python
RVE_SIZE = [1.5, 1.5, 2.0]
FIBER_RADIUS = 0.05
TARGET_VF = 0.45
GLOBAL_SEED_SIZE = 0.03
RSA_SEEDING_RATIO = 0.1  # Creates clustering
```
**Time**: ~30 minutes  
**Use**: Manufacturing defect studies

### Example 4: Fine Mesh Analysis

```python
RVE_SIZE = [0.8, 0.8, 1.5]
FIBER_RADIUS = 0.04
TARGET_VF = 0.50
GLOBAL_SEED_SIZE = 0.015  # Very fine
DEVIATION_FACTOR = 0.05
```
**Time**: ~2 hours  
**Use**: High-accuracy simulations  
**Note**: Requires 32+ GB RAM

## ⚠️ Important Notes

### Performance Considerations

1. **Mesh Size Impact**
   - Halving mesh size → 8× more elements → 8-64× longer analysis time
   - Start coarse, refine gradually
   - Monitor memory usage

2. **Volume Fraction Limits**
   - Recommended: Vf < 0.65
   - Vf > 0.7: Very challenging, may fail
   - Consider manufacturing feasibility

3. **Model Size**
   - Small RVE (0.5×0.5×1.0): ~50K elements
   - Medium RVE (1.0×1.0×2.0): ~200K elements
   - Large RVE (2.0×2.0×4.0): ~1.6M elements

### Best Practices

1. **Start Small**
   - Test with small RVE and coarse mesh
   - Validate results before scaling up
   - Use Vf = 0.3-0.4 for initial tests

2. **Mesh Quality**
   - Check mesh before analysis
   - Look for distorted elements
   - Adjust DEVIATION_FACTOR if needed

3. **Memory Management**
   - 16 GB RAM: Small-medium models
   - 32 GB RAM: Medium-large models
   - 64 GB RAM: Large models with fine mesh

4. **Material Orientation**
   - Fiber Direction-1 is always aligned with +Z
   - Verify orientation in ABAQUS viewer
   - Adjust loads/BCs accordingly

5. **Cohesive Elements**
   - Significantly increase element count
   - May cause convergence issues if poorly calibrated
   - Test without cohesive first

## 🔍 Troubleshooting

### Issue 1: Cannot Achieve Target Vf

**Symptoms**: Script terminates with volume fraction error

**Solutions**:
```python
# Option A: Reduce target Vf
TARGET_VF = 0.45  # Instead of 0.65

# Option B: Increase RVE size
RVE_SIZE = [1.5, 1.5, 2.0]  # Instead of [1.0, 1.0, 2.0]

# Option C: Decrease fiber radius
FIBER_RADIUS = 0.04  # Instead of 0.05

# Option D: Reduce minimum distance
MIN_DIST_FACTOR = 0.005  # Instead of 0.01
```

### Issue 2: Mesh Generation Takes Forever

**Symptoms**: Script hangs during meshing

**Solutions**:
```python
# Increase mesh seed size
GLOBAL_SEED_SIZE = 0.04  # Coarser mesh

# Adjust deviation factor
DEVIATION_FACTOR = 0.15  # Allow more deviation

# Reduce RVE size for testing
RVE_SIZE = [0.5, 0.5, 1.0]
```

### Issue 3: Out of Memory

**Symptoms**: ABAQUS crashes or system freezes

**Solutions**:
1. Close other applications
2. Reduce model size or mesh density
3. Use 64-bit ABAQUS
4. Increase virtual memory
5. Consider cluster computing

### Issue 4: PBC Pairing Failures

**Symptoms**: "Failed to pair nodes" errors

**Solutions**:
```python
# Increase pairing tolerance
PAIRING_TOLERANCE_FACTOR = 8.0  # Instead of 5.0

# Adjust mesh seed size
GLOBAL_SEED_SIZE = 0.025  # Try different values
```

### Issue 5: Analysis Convergence Problems

**Symptoms**: ABAQUS analysis doesn't converge

**Solutions**:
```python
# Adjust cohesive stabilization
COHESIVE_STAB_COEFF = 0.001  # Increase

# Modify damage parameters
MATRIX_DAMAGE_DISPLACEMENT = 0.0001  # Increase

# Check material properties
# Ensure realistic values for all parameters
```

## 📈 Computational Cost Estimates

| Model Size | Elements | Mesh Time | Analysis Time* | RAM Required |
|------------|----------|-----------|----------------|--------------|
| Small      | ~50K     | 5-10 min  | 1-2 hours      | 8 GB         |
| Medium     | ~200K    | 15-30 min | 4-8 hours      | 16 GB        |
| Large      | ~800K    | 45-90 min | 1-2 days       | 32 GB        |
| X-Large    | ~2M      | 2-4 hours | 3-7 days       | 64 GB        |

*Analysis time for standard static or quasi-static loading

## 🎓 Tips for Success

1. **Incremental Approach**
   - Start: Small RVE, coarse mesh, low Vf
   - Validate: Check results, mesh quality
   - Refine: Gradually increase complexity

2. **Parameter Documentation**
   - Record all parameter combinations
   - Note generation time and success
   - Build a database of working configurations

3. **Result Validation**
   - Compare with analytical predictions
   - Check stress distribution patterns
   - Verify effective properties

4. **Parallel Processing**
   - Use ABAQUS parallel execution
   - Set appropriate CPU count
   - Monitor CPU and memory usage

5. **Data Management**
   - Models can be 1-10 GB each
   - Organize by Vf, mesh size, etc.
   - Delete unnecessary intermediate files

## 📚 Technical Background

### Algorithm Details

The script uses a sophisticated three-phase algorithm:

1. **RSA Seeding**
   - Fast initial placement
   - Ensures no overlap
   - Controlled by RSA_SEEDING_RATIO

2. **Anchored Relaxation**
   - Iterative position refinement
   - Maintains minimum distance
   - Achieves target volume fraction

3. **Forced Correction**
   - Final adjustment phase
   - Handles boundary conditions
   - Ensures periodicity

### Material Orientation System

- **Direction-1**: Fiber axis (Z-direction)
- **Direction-2**: Transverse direction 1 (arbitrary in XY)
- **Direction-3**: Transverse direction 2 (perpendicular to 1 and 2)

For orthotropic material:
- E1 = Longitudinal modulus (along fiber)
- E2 = E3 = Transverse modulus
- G12 = G13 = Longitudinal shear
- G23 = Transverse shear

### Periodic Boundary Conditions

Three reference points control periodicity:
- **Ref-X**: Left-Right faces (X-direction)
- **Ref-Y**: Front-Back faces (Y-direction)
- **Ref-Z**: Bottom-Top faces (Z-direction)

Constraint equations: `u_slave - u_master + u_ref = 0`

## 📝 Citation

If you use this code in your research:

```bibtex
@software{3d_rve_model_2025,
  author = {Liu, Zhengpeng},
  title = {3D RVE Model Generator for ABAQUS},
  year = {2025},
  url = {https://github.com/yourusername/3D_RVE_Model},
  version = {1.0}
}
```

## 👤 Author

**Liu Zhengpeng (刘正鹏)**
- GitHub: [@小盆i](https://github.com/yourusername)
- Email: 1370872708@qq.com / Zhengpeng0105@gmail.com
- Technical Blog: CSDN/知乎 @小盆i

## 📄 License

This project is licensed under the MIT License.

## 🙏 Acknowledgments

- Composite micromechanics research community
- ABAQUS scripting community
- All users providing feedback

## 📮 Support

- **Issues**: Report bugs via GitHub Issues
- **Email**: Zhengpeng0105@gmail.com
- **Documentation**: See inline comments in code

---

<a name="chinese"></a>

# ABAQUS三维RVE建模工具

## 📋 项目概述

这是一个强大的Python脚本,用于在ABAQUS中自动生成具有可控纤维体积分数的**三维代表性体积单元(RVE)**模型。该工具将2D RVE生成器扩展到三维,通过连续的圆柱形纤维实现真实的复合材料微观力学仿真。

### ✨ 核心功能

- **三维纤维结构**:沿Z轴方向的连续圆柱形纤维
- **可控体积分数**:根据用户设定的纤维体积分数(Vf)自动生成
- **先进布置算法**:混合算法(RSA播种+锚定松弛+强制校正)实现XY平面内最优纤维分布
- **正交各向异性纤维**:真实的纤维性能,具有方向性刚度(E1, E2, G12, G23, ν12)
- **最小距离保证**:确保纤维间距(XY平面内),并报错
- **完整材料系统**:自动创建纤维(正交各向异性)+基体(Drucker-Prager)+界面(内聚力)
- **三维周期性边界条件**:XYZ三个方向的周期性,准确模拟RVE行为
- **三维内聚力单元**:纤维-基体界面的COH3D8单元
- **精确相识别**:几何算法准确识别纤维体和基体体
- **自动材料取向**:纤维材料方向1对齐至+Z轴
- **CSV导出**:纤维中心坐标和几何数据
- **距离验证**:验证XY平面内纤维间距并提供统计信息

## 🔬 3D与2D对比

| 特性 | 2D RVE | 3D RVE (本脚本) |
|------|--------|----------------|
| **纤维几何** | 圆形截面 | 沿Z轴的连续圆柱体 |
| **纤维材料** | 各向同性 | 正交各向异性(真实) |
| **周期性边界条件** | XY方向 | XYZ方向 |
| **内聚力单元** | COH2D4 | COH3D8 |
| **距离检查** | XY平面 | 仅XY平面(Z连续) |
| **网格复杂度** | 基准 | 10-50倍 |
| **计算成本** | 中等 | 显著更高 |
| **真实性** | 适合2D分析 | 更接近真实材料 |

## 🎯 应用场景

- 单向复合材料仿真
- 纤维增强复合材料微观力学
- 界面脱粘和损伤研究
- 有效性能预测
- 渐进损伤分析
- 多尺度建模
- 复合材料制造工艺仿真

## 🔧 系统要求

### 软件要求
- **ABAQUS**:2023版本或兼容版本
- **Python**:2.7(ABAQUS内置)

### 硬件推荐
- **内存**:最少16 GB(推荐32 GB用于大型模型)
- **CPU**:多核处理器(推荐8核以上)
- **磁盘**:每个模型5-10 GB可用空间
- **GPU**:不需要(ABAQUS基于CPU)

### Python模块
所有必需模块均包含在ABAQUS安装中。

## 📥 安装说明

1. **克隆或下载**
   ```bash
   git clone https://github.com/yourusername/3D_RVE_Model.git
   cd 3D_RVE_Model
   ```

2. **验证ABAQUS**
   ```bash
   abaqus information=system
   ```

## 🚀 快速开始

### 基本使用

1. **编辑参数**(第1747-1960行):
   ```python
   # 基本几何
   RVE_SIZE = [1.0, 1.0, 2.0]     # [宽度,高度,深度],单位mm
   FIBER_RADIUS = 0.05             # 纤维半径,单位mm
   TARGET_VF = 0.50                # 目标体积分数(0.0-1.0)
   
   # 网格设置
   GLOBAL_SEED_SIZE = 0.02         # 全局网格种子尺寸
   ```

2. **在ABAQUS中运行**
   ```bash
   # 命令行(无GUI)
   abaqus cae noGUI=3D_RVE_Model.py
   
   # 或在ABAQUS CAE GUI中
   文件 → 运行脚本 → 选择3D_RVE_Model.py
   ```

3. **预期时间**
   - 简单模型(Vf=0.3,小尺寸):5-15分钟
   - 标准模型(Vf=0.5,中等尺寸):15-45分钟
   - 复杂模型(Vf=0.6,大尺寸):1-3小时

## 📖 参数指南

### 几何参数

```python
# RVE_SIZE:[宽度,高度,深度],单位mm
# - 宽度和高度:XY平面尺寸(纤维分布区域)
# - 深度:Z轴长度(纤维长度)
# - 典型值:[1.0, 1.0, 2.0]用于长宽比测试
RVE_SIZE = [1.0, 1.0, 2.0]

# FIBER_RADIUS:圆柱形纤维的半径,单位mm
# - 典型值:0.003-0.1 mm,取决于纤维类型
# - 碳纤维:约0.003-0.005 mm
# - 玻璃纤维:约0.005-0.01 mm
FIBER_RADIUS = 0.05

# TARGET_VF:目标纤维体积分数
# - 范围:0.0到1.0(推荐0.3-0.6)
# - 典型复合材料:0.5-0.65
# - 高性能:最高0.7(具有挑战性)
TARGET_VF = 0.50

# MIN_DIST_FACTOR:最小距离系数
# - 乘以纤维直径得到最小间距
# - 范围:0.001-0.05
# - 更小:更密集填充,更难划分网格
# - 更大:更稀疏分布,更易划分网格
MIN_DIST_FACTOR = 0.01
```

### 网格参数

```python
# GLOBAL_SEED_SIZE:单元尺寸,单位mm
# - 对3D模型至关重要(显著影响计算量)
# - 更小:更准确,更慢
# - 典型范围:0.01-0.05 mm
# - 建议:从0.03开始,根据需要细化
GLOBAL_SEED_SIZE = 0.02

# DEVIATION_FACTOR:网格偏差容差
# - 控制曲面上的网格质量
# - 范围:0.05-0.2
# - 更小:更好的质量,更多单元
DEVIATION_FACTOR = 0.1

# MIN_SIZE_FACTOR:最小单元尺寸系数
# - 防止产生极小的单元
# - 乘以全局种子尺寸
# - 范围:0.05-0.5
MIN_SIZE_FACTOR = 0.1

# PAIRING_TOLERANCE_FACTOR:PBC节点配对容差
# - 乘以全局种子尺寸
# - 如果PBC配对失败则调整
# - 典型值:3.0-10.0
PAIRING_TOLERANCE_FACTOR = 5.0
```

### 纤维分布控制

```python
# RSA_SEEDING_RATIO:控制纤维布置算法
# 
# 高值(0.8-1.0) - "快速均匀模式":
#   - 快速生成
#   - 均匀分布
#   - 推荐用于生产
#   - 示例:0.9 = 90% RSA, 10%松弛
#
# 低值(0.0-0.3) - "物理团簇模式":
#   - 真实团簇效应
#   - 较慢生成
#   - 更接近真实材料
#   - 示例:0.1 = 10% RSA, 90%松弛
#
# 中等值(0.4-0.7) - "平衡模式":
#   - 两种方法的混合
#   - 中等速度
#   - 示例:0.5 = 50% RSA, 50%松弛

RSA_SEEDING_RATIO = 0.9
```

### 材料参数

#### 纤维材料(正交各向异性)

```python
# 纵向性能(沿纤维轴=Z轴)
FIBER_E1 = 235.0      # 纵向模量(GPa)
FIBER_NU12 = 0.2      # 主泊松比

# 横向性能(垂直于纤维轴)
FIBER_E2 = 15.0       # 横向模量(GPa)

# 剪切性能
FIBER_G12 = 15.0      # 纵向剪切模量(GPa)
FIBER_G23 = 7.0       # 横向剪切模量(GPa)

# 典型值:
# - 碳纤维:E1约235 GPa, E2约15 GPa, G12约15 GPa
# - 玻璃纤维:E1约72 GPa, E2约72 GPa, G12约30 GPa
```

#### 基体材料(Drucker-Prager)

```python
MATRIX_E = 3170.0                      # 弹性模量(MPa)
MATRIX_NU = 0.35                       # 泊松比
MATRIX_FRICTION_ANGLE = 16.0           # 摩擦角(度)
MATRIX_FLOW_STRESS_RATIO = 1.0         # 流动应力比
MATRIX_DILATION_ANGLE = 16.0           # 膨胀角(度)
MATRIX_HARDENING_YIELD = 106.4         # 屈服应力(MPa)
MATRIX_HARDENING_PLASTIC_STRAIN = 0.0  # 屈服时的塑性应变
MATRIX_DAMAGE_STRAIN = 0.01            # 损伤起始应变
MATRIX_DAMAGE_DISPLACEMENT = 5e-05     # 损伤位移(mm)
```

#### 内聚力界面材料

```python
# 刚度(N/mm³)
COHESIVE_K_NN = 1e8    # 法向刚度
COHESIVE_K_SS = 1e8    # 第一切向刚度
COHESIVE_K_TT = 1e8    # 第二切向刚度

# 强度(MPa)
COHESIVE_T_N = 44.0    # 法向强度
COHESIVE_T_S = 82.0    # 第一切向强度
COHESIVE_T_T = 82.0    # 第二切向强度

# 断裂能(N/mm)
COHESIVE_GIC = 0.001   # I型
COHESIVE_GIIC = 0.002  # II型
COHESIVE_GIIIC = 0.002 # III型

# BK准则
COHESIVE_ETA = 1.5     # 指数(1.0-3.0)

# 数值稳定性
COHESIVE_STAB_COEFF = 0.0001
```

## 📊 输出文件

### ABAQUS模型
- **模型名称**:`3D-RVE-Vf-XX`(XX为体积分数百分比)
- **内容**:
  - 三维圆柱形纤维几何
  - 带取向的材料定义
  - 内聚力界面单元
  - 三维周期性边界条件(XYZ)
  - 已划分网格的装配体,可直接分析

### CSV导出
- **文件名**:`FiberCenters_3D_VfXX_YYYYMMDD_HHMMSS.csv`
- **格式**:
  ```csv
  # 3D RVE Fiber Center Coordinates
  # RVE Size (Width x Height x Depth): 1.000000 x 1.000000 x 2.000000
  # Fiber Radius: 0.050000
  # Fiber Length: 2.000000
  # Target Volume Fraction: 0.5000
  # Total Fiber Count: 127
  Fiber_ID,X_Coordinate,Y_Coordinate,Z_Start,Z_End
  1,0.12345678,0.23456789,0.00000000,2.00000000
  2,0.34567890,0.45678901,0.00000000,2.00000000
  ...
  ```

## 💡 使用示例

### 示例1:标准单向复合材料

```python
RVE_SIZE = [1.0, 1.0, 2.0]
FIBER_RADIUS = 0.05
TARGET_VF = 0.50
GLOBAL_SEED_SIZE = 0.02
RSA_SEEDING_RATIO = 0.9

# 碳纤维性能
FIBER_E1 = 235.0
FIBER_E2 = 15.0
FIBER_G12 = 15.0
FIBER_G23 = 7.0
FIBER_NU12 = 0.2

# 环氧基体
MATRIX_E = 3500.0
MATRIX_NU = 0.35
```
**时间**:约20分钟  
**用途**:通用复合材料测试

### 示例2:高纤维含量

```python
RVE_SIZE = [1.0, 1.0, 1.5]
FIBER_RADIUS = 0.04
TARGET_VF = 0.65
GLOBAL_SEED_SIZE = 0.025
MIN_DIST_FACTOR = 0.005
RSA_SEEDING_RATIO = 0.7
```
**时间**:约45分钟  
**用途**:高性能复合材料

### 示例3:真实团簇效应

```python
RVE_SIZE = [1.5, 1.5, 2.0]
FIBER_RADIUS = 0.05
TARGET_VF = 0.45
GLOBAL_SEED_SIZE = 0.03
RSA_SEEDING_RATIO = 0.1  # 创建团簇
```
**时间**:约30分钟  
**用途**:制造缺陷研究

### 示例4:精细网格分析

```python
RVE_SIZE = [0.8, 0.8, 1.5]
FIBER_RADIUS = 0.04
TARGET_VF = 0.50
GLOBAL_SEED_SIZE = 0.015  # 非常精细
DEVIATION_FACTOR = 0.05
```
**时间**:约2小时  
**用途**:高精度仿真  
**注意**:需要32+ GB内存

## ⚠️ 重要说明

### 性能考虑

1. **网格尺寸影响**
   - 网格尺寸减半→单元数增加8倍→分析时间增加8-64倍
   - 从粗网格开始,逐步细化
   - 监控内存使用

2. **体积分数限制**
   - 推荐:Vf < 0.65
   - Vf > 0.7:非常具有挑战性,可能失败
   - 考虑制造可行性

3. **模型尺寸**
   - 小型RVE(0.5×0.5×1.0):约5万单元
   - 中型RVE(1.0×1.0×2.0):约20万单元
   - 大型RVE(2.0×2.0×4.0):约160万单元

### 最佳实践

1. **从小开始**
   - 用小RVE和粗网格测试
   - 放大前验证结果
   - 初始测试使用Vf = 0.3-0.4

2. **网格质量**
   - 分析前检查网格
   - 查找扭曲单元
   - 必要时调整DEVIATION_FACTOR

3. **内存管理**
   - 16 GB内存:小-中型模型
   - 32 GB内存:中-大型模型
   - 64 GB内存:大型模型与精细网格

4. **材料取向**
   - 纤维方向1始终与+Z对齐
   - 在ABAQUS查看器中验证取向
   - 相应调整载荷/边界条件

5. **内聚力单元**
   - 显著增加单元数量
   - 如果校准不当可能导致收敛问题
   - 先不用内聚力测试

## 🔍 故障排除

### 问题1:无法达到目标Vf

**症状**:脚本因体积分数错误而终止

**解决方案**:
```python
# 选项A:降低目标Vf
TARGET_VF = 0.45  # 而不是0.65

# 选项B:增加RVE尺寸
RVE_SIZE = [1.5, 1.5, 2.0]  # 而不是[1.0, 1.0, 2.0]

# 选项C:减小纤维半径
FIBER_RADIUS = 0.04  # 而不是0.05

# 选项D:减小最小距离
MIN_DIST_FACTOR = 0.005  # 而不是0.01
```

### 问题2:网格生成耗时过长

**症状**:脚本在网格生成时挂起

**解决方案**:
```python
# 增加网格种子尺寸
GLOBAL_SEED_SIZE = 0.04  # 更粗的网格

# 调整偏差因子
DEVIATION_FACTOR = 0.15  # 允许更多偏差

# 减小RVE尺寸用于测试
RVE_SIZE = [0.5, 0.5, 1.0]
```

### 问题3:内存不足

**症状**:ABAQUS崩溃或系统冻结

**解决方案**:
1. 关闭其他应用程序
2. 减小模型尺寸或网格密度
3. 使用64位ABAQUS
4. 增加虚拟内存
5. 考虑集群计算

### 问题4:PBC配对失败

**症状**:"Failed to pair nodes"错误

**解决方案**:
```python
# 增加配对容差
PAIRING_TOLERANCE_FACTOR = 8.0  # 而不是5.0

# 调整网格种子尺寸
GLOBAL_SEED_SIZE = 0.025  # 尝试不同值
```

### 问题5:分析收敛问题

**症状**:ABAQUS分析不收敛

**解决方案**:
```python
# 调整内聚力稳定化
COHESIVE_STAB_COEFF = 0.001  # 增加

# 修改损伤参数
MATRIX_DAMAGE_DISPLACEMENT = 0.0001  # 增加

# 检查材料性能
# 确保所有参数都是真实值
```

## 📈 计算成本估算

| 模型尺寸 | 单元数 | 网格时间 | 分析时间* | 所需内存 |
|---------|--------|---------|----------|---------|
| 小型     | 约5万   | 5-10分钟 | 1-2小时   | 8 GB    |
| 中型     | 约20万  | 15-30分钟| 4-8小时   | 16 GB   |
| 大型     | 约80万  | 45-90分钟| 1-2天     | 32 GB   |
| 超大型   | 约200万 | 2-4小时  | 3-7天     | 64 GB   |

*标准静态或准静态加载的分析时间

## 🎓 成功技巧

1. **渐进方法**
   - 开始:小RVE,粗网格,低Vf
   - 验证:检查结果,网格质量
   - 细化:逐步增加复杂度

2. **参数记录**
   - 记录所有参数组合
   - 注明生成时间和成功与否
   - 建立有效配置数据库

3. **结果验证**
   - 与解析预测比较
   - 检查应力分布模式
   - 验证有效性能

4. **并行处理**
   - 使用ABAQUS并行执行
   - 设置适当的CPU数量
   - 监控CPU和内存使用

5. **数据管理**
   - 每个模型可能1-10 GB
   - 按Vf、网格尺寸等组织
   - 删除不必要的中间文件

## 📚 技术背景

### 算法细节

脚本使用复杂的三阶段算法:

1. **RSA播种**
   - 快速初始布置
   - 确保无重叠
   - 由RSA_SEEDING_RATIO控制

2. **锚定松弛**
   - 迭代位置细化
   - 保持最小距离
   - 达到目标体积分数

3. **强制校正**
   - 最终调整阶段
   - 处理边界条件
   - 确保周期性

### 材料取向系统

- **方向1**:纤维轴(Z方向)
- **方向2**:横向方向1(XY平面内任意)
- **方向3**:横向方向2(垂直于1和2)

对于正交各向异性材料:
- E1 = 纵向模量(沿纤维)
- E2 = E3 = 横向模量
- G12 = G13 = 纵向剪切
- G23 = 横向剪切

### 周期性边界条件

三个参考点控制周期性:
- **Ref-X**:左-右面(X方向)
- **Ref-Y**:前-后面(Y方向)
- **Ref-Z**:底-顶面(Z方向)

约束方程:`u_slave - u_master + u_ref = 0`

## 📝 引用

如果您在研究中使用此代码:

```bibtex
@software{3d_rve_model_2025,
  author = {刘正鹏},
  title = {ABAQUS三维RVE建模工具},
  year = {2025},
  url = {https://github.com/yourusername/3D_RVE_Model},
  version = {1.0}
}
```

## 👤 作者

**刘正鹏 (Liu Zhengpeng)**
- GitHub: [@小盆i](https://github.com/yourusername)
- 邮箱: 1370872708@qq.com / Zhengpeng0105@gmail.com
- 技术博客: CSDN/知乎 @小盆i

## 📄 许可证

本项目采用MIT许可证。

## 🙏 致谢

- 复合材料微观力学研究社区
- ABAQUS脚本编程社区
- 所有提供反馈的用户

## 📮 支持

- **问题反馈**:通过GitHub Issues报告错误
- **邮箱**:Zhengpeng0105@gmail.com
- **文档**:查看代码中的内联注释

---

**版本**: v1.0  
**最后更新**: 2025-10-20  
**维护状态**: 活跃维护中

## 🔗 相关项目

- [2D RVE Model Generator](https://github.com/ZPL-03/2D_RVE_Model) - 二维版本

---

**提示**: 3D模型计算量显著大于2D模型,请根据您的硬件资源合理设置参数。从小模型和粗网格开始,逐步提高复杂度。

**Tip**: 3D models are computationally much more expensive than 2D models. Set parameters according to your hardware resources. Start with small models and coarse meshes, gradually increasing complexity.
