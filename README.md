# 大规模三维道路生成技术

## 输入
- **input**:   [OSM File](https://wiki.openstreetmap.org/wiki/OSM_file_formats)
- **format**:
``` xml
<node id="299722213" visible="true" version="3" changeset="95219677" timestamp="2020-12-03T11:01:49Z" user="Ometepe" uid="10019169" lat="30.4710541" lon="117.1974277"/>
<way id="773575283" visible="true" version="1" changeset="81126753" timestamp="2020-02-17T16:38:06Z" user="Lepuse" uid="9560399">
  <nd ref="7220742943"/>
  <nd ref="7220742904"/>
  <nd ref="7220720185"/>
  <nd ref="7220676884"/>
  <nd ref="7220676883"/>
  <nd ref="7220676882"/>
  <nd ref="7220676881"/>
  <nd ref="7220720189"/>
  <nd ref="7220720190"/>
  <nd ref="7220720191"/>
  <nd ref="7220720192"/>
  <nd ref="3159587925"/>
  <nd ref="3159587922"/>
  <nd ref="3159587917"/>
  <nd ref="3159587904"/>
  <nd ref="3159587901"/>
  <nd ref="3159587900"/>
  <nd ref="3159587899"/>
 </way>
```
- Input format introduction：输入包括node节点和way节点，node表示点，way节点中的nd节点ref属性来自于node的ID，表示一条道路.

## 坐标转换
二维坐标向三维坐标的转换可以通过（lon, lat）获取搞成，我们的系统是
```cpp
double alt = ViWoROOT::GetTerrainInterface()->GetPreciseElevation(lon, lat);
```
经纬高坐标必须转换成每个系统坐标系的全局坐标```（lon, lat, alt） -> (x, y, z)```.
本文的全局坐标系统是地球球坐标系统，最后需要转换成局部坐标计算

**局部坐标系** 
![局部坐标系](https://github.com/yangpei11/RoadGraph-C-/blob/master/Figure/1.png  "局部坐标系")

## 数学建模

**折线道路** 
![道路折线](https://github.com/yangpei11/RoadGraph-C-/blob/master/Figure/2.png)

**平滑道路**
![平滑后的折线](https://github.com/yangpei11/RoadGraph-C-/blob/master/Figure/3.png)
OSM中道路节点连接起来就是折现线段，然后现实生活中基本都是平滑的曲线道路。根据美国公路标准，道路大部分由圆曲线，直线，回旋曲线或者园曲线，直线组成，由于回旋曲线在工程上要求较高，因此算法采用后者。因此数学建模问题为找到一系列的```{r1,r2,r3,...,rn}```, 算法根据优化式```maxmin(ri)```可以求解,具体详见```alpha_assign```函数

二维道路到三维道路的转换：给二维的每个离散点赋予C2连续的高度“翘起来”，即可三维化，三维化采用三次样条技术，详见代码。

## 绘制策略

**平滑道路的三角面片组织**
![平滑道路的三角面片组织](https://github.com/yangpei11/RoadGraph-C-/blob/master/Figure/4.png)

**绘制效果**
![绘制效果](https://github.com/yangpei11/RoadGraph-C-/blob/master/Figure/5.png)

## examples
![fig1](https://github.com/yangpei11/RoadGraph-C-/blob/master/Figure/example1.png)

**桥与地面接触的细节**
![fig2](https://github.com/yangpei11/RoadGraph-C-/blob/master/Figure/example2.png)

**复杂的立交桥**
![fig3](https://github.com/yangpei11/RoadGraph-C-/blob/master/Figure/example3.png)

**拱桥**
![fig4](https://github.com/yangpei11/RoadGraph-C-/blob/master/Figure/example4.png)

**复杂道路**
![fig5](https://github.com/yangpei11/RoadGraph-C-/blob/master/Figure/s.jpg)

**与谷歌卫星影像的对比**
![fig6](https://github.com/yangpei11/RoadGraph-C-/blob/master/Figure/example5.png)



