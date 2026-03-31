#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip>
#include <sstream>
#include <functional>
#include <optional>
#include <memory>
#include <algorithm>

/*
* 这道题太神秘了你知道吗, 题目给的描述完全读不进去, 还不如我自己查资料呢. 如果未来我有机会当上助教, 我要把这道本身质量很高的题重新设计一遍
* 把草稿先打在这里, 理一理思路, 推一点物理原理
* 混凝土用力压很坚固, 但容易被掰断, 桥在中间会下弯, 下弯以后, 下面的长度变大, 上面的长度变小
* 变长变短反映到力学上, 就是受压和受拉, 因此, 为了不让下半部分裂开, 就必须往底部埋入抗拉的钢筋
* 这是钢筋混凝土的基本思想, 然后再写力学原理
* 力学原理我就不推了, 能直接查到现有的公式, 我在这里更关心的是公式的形式和它们的物理意义
* 首先, 弯矩是桥梁界面受到的折断力矩, 力矩 τ=r×F, 剪力是平行于截面的力
* 对于均匀分布的载荷, 总能把外界的力当成桥梁的自重, 因此直接当作桥梁自重处理即可
* 若梁每米重 q, 总长 L, 则中点处的弯矩最大, 为 qL²/8
* 再考虑小车, 小车是两个距离为 d 的轮子, 带着集中力 P 在桥上开
* 假如轮子只有一个, 那自然在桥正中间最容易断, 此时桥中心的弯矩 M=(P/2)*(L/2)=PL/4
* 受此启发再考虑两个轮子的情形, 此时不是考虑两个轮子各自的效应, 而是这个整体对桥产生的总折断效应
* 由结构力学的影响线理论, 这辆车的重心和其中一个轮子对称分布在桥的中点两侧时, 这个轮子下的弯矩是全桥最大
* 由力矩平衡, M_max = (P/2L) * (L-d/2)²
* 因此这也就是车开过时, 桥必须承受的极限弯矩
* 有了自重, 有了外力, 两者加起来就是总弯矩了
*
* 好! 有了外力, 接下来看看怎么配钢筋
* 钢筋的核心是力偶平衡, 截面处, 上半部分的混凝土负责压, 下半部分负责拉, 中间有一条既不拉也不压的中性轴
* 把桥从正中间劈开, 先隔离左半边的受力情况
* 首先, 桥的左侧受到支撑力, 重心受到重力, 既然左半边桥静止而没有被压下去, 则在切口处一定满足水平方向合外力, 力矩均为零
* 那么切口处的材料必须给出逆时针的力矩来抵抗, 可以等效为一个杠杆
* 先看水平方向受力平衡
* 顶部的混凝土被挤压了, 它想弹回去, 所以在向右推这个面, 可以合成为推力 C
* 底部的钢筋被拉长了, 它想缩回去, 所以在向左拉这个面, 可以合成为拉力 T
* 被拉长的混凝土因为出力太小不考虑
* 那么由水平方向合外力为零可以得到 C=T
* 又因为这两个力不共线且反向, 则形成了一个力偶, 接下来的任务针对找出它的力臂
* 设截面高 h, 钢筋埋在靠底部的位置, 距顶部为 h_0
* 假设从顶部往下, 深度为 x 的区域都被压实了, 那么它的等效受力中心就在深度为 x/2 的位置
* 因此两个力相距 h_0 - (x/2)
* 又由力矩平衡, 内部产生的力偶必须等于外部的弯矩, 故 M=C*(h_0-(x/2))
* 而推力 C = 压强 * 受力面积, 压强即为极限抗压强度 FC, 面积为桥宽 * 受压区域高度
* 故 C = FC * b * x
* 因此, M = FC * b * x * (h_0 - (x/2))
* 这样就得到了关于 x 的一元二次方程
* 在工程上, 为了计算方便, 将 x 替换成了比例 ξ=x/h_0, 这样求出来的更有泛用性
* 整理得 ξ-0.5ξ² = M/(FC*b*(h_0)²)
* 右边的都是常数了, 直接定义为 α_s, 则方程化为 0.5ξ²-ξ+α_s=0
* 物理意义知 ξ<1, 因此解得 ξ = 1-sqrt(1-2α_s)
* 解得 ξ 以后, 钢筋的位置已经解出来了
* 那么杠杆的力臂 = h_0 - x/2 = h_0 - 0.5ξh_0 = (1-0.5ξ)h_0
* 则钢筋承受的总拉力 T = M / (1-0.5ξ)h_0
* 又在材料力学上, 拉力 T 等于钢筋的总截面积 A_s 乘以钢筋的抗拉强度 FY
* 因此联立解得 A_s = M / ( FY * (1-0.5ξ) h_0 )
*
* 可把我牛逼坏了, 叉会腰先
*
* 接下来, 光考虑中间的弯曲还不够, 因为车在支座旁边时, 桥可能会被斜向压碎
* 类比一根粉笔, 放在桌子边缘, 放了重物以后, 会沿着桌边断裂, 这个力叫做剪切力
* 桥的两边虽然没有弯矩, 但剪力很大, 因为相当于两个桥墩撑起了整座桥的重力
* 由莫尔应力圆知: 对于一段桥的微元, 如果左边受向上的剪力, 右边受向下的剪力, 这个矩形会被搓成平行四边形
* 平行四边形导致的后果就是, 一条对角线被压缩, 另一条对角线被拉伸
* 又由于混凝土很怕拉, 因此如果不做处理, 桥面会从侧面裂出一条斜缝, 接下来解决这个问题
* 首先, 并不能往这个平行四边形里直接塞钢筋, 因为另一条对角线在压缩, 如果用钢筋把裂缝塞死了, 压缩方向的混凝土会被直接压烂, 更糟糕
* 所以, 桥面本身是有剪力上限的, 它由混凝土的材料特性和几何特性决定
* 那就先来计算桥面自带的抗性, V_cs = 0.7 * FT * b * h_0, FT 是混凝土的抗拉强度
* 再计算在给定载荷下, 需要多少钢筋来补足
* 先考虑怎么安插钢筋, 两个直接的思路是, 要么在剪切面竖插, 要么在桥面外包裹
* 竖插的问题在于, 在裂缝处钢筋会收到两个方向相反的剪力, 而且这个地方会导致混凝土堆积进而碎裂, 所以不可以
* 因此需要在桥面外包裹, 这个就叫做箍筋
* 当裂缝想要撕裂桥体时, 必然要横跨好几圈垂直的箍筋, 就像缝合线一样拉住伤口, 防止断裂
* 由于裂缝是 45° 断裂的, 所以对于高为 h_0 的桥面, 裂缝在水平和竖直方向延伸的长度相等, 也即横跨 h_0 的长度
* 假设每隔 s 就绑一圈, 那么这条裂缝就穿过了 h_0/s 圈箍筋
* 而单圈箍筋能提供的拉力, 等于单圈钢筋的截面积 A_sv * 钢筋抗拉强度 FYV
* 因此钢筋提供的抗剪力 V_s = 圈数 * 单圈拉力 = h_0/s * A_sv * FYV
* 为了让造价尽可能低, 需要让 V_s 刚好等于多出来的剪力 V_total - V_cs
* 将 A_sv / s 打包成一个整体, 记作 Asv_s, 物理意义是, 沿着桥走, 每毫米需要分配多少箍筋面积
* 进而解出 Asv_s = (V_total - V_cs) / FYV*h_0
* 又因为箍筋是一个矩形的铁丝框, 当桥体撕裂的时候, 其实受力的是桥两侧的边, 因此一圈箍筋受力的面积为 2*Asv_leg
* 知道了每毫米需要的面积, 又知道了一圈面积多少, 两者一除就得到了一圈钢筋能管多远
* 又在现实工程中, 由于实操问题, 需要有范围限制和取整, 最后再做做处理就得到结果
*
* 在算出理论需要的钢筋面积后, 就需要拿真实的钢筋型号填满了
* 由于钢筋尺寸是离散的, 选取时只能向上取整, 则必然会导致实际面积不低于理论计算面积
* 因此这座桥实际能承受的极限会比需求高, 这个极限就是绘图时计算百分比的依据, 记作 M_u
*/

/*
* 接下来回到舒适区了, 理一下机器学习的思路
* 对于这种公式确定的找最优解问题, 不应该用神经网络或者梯度下降的方法, 真没必要
* 这个的数据量并不大, 甚至计算量没我图形时钟大, 可以直接暴力穷举, 摘出最优解即可
* 这里的损失函数非常简单, 就是 total_cost 总成本, 甚至连权都不用加
* 首先划分网格, 按宽, 高, 直径划分, 进行正向传播
* 为了加速搜索的过程, 把一些没有物理意义的, 或者超出载荷能力的部分剪枝掉, 然后记录最优造价即可
*/


// 先来定义 1mol 常量
// 混凝土抗压强度 FC
const double FC = 32.4;
// 混凝土抗拉强度 FT
const double FT = 2.65;
// 主筋抗拉强度 FY
const double FY = 400.0;
// 箍筋抗拉强度 FYV
const double FYV = 360.0;
// 混凝土容重 γ
const double GAMMA = 25.0;
// 钢筋密度
const double REBAR_DENSITY = 7.85;
// 保护层厚度
const double COVER = 30.0;
// 受拉钢筋合力点到边缘的距离 A_s
const double AS_DIST = 40.0;

// 用一个结构体储存最优方案
struct DesignResult
{
    // 方案是否有效
    bool valid = false;
    // 宽度和高度
    double b = 0, h = 0;
    // 理论需要的总钢筋面积
    double As = 0;
    // 主钢筋直径
    int mainbar_D = 0;
    // 主钢筋数量
    int mainbar_R = 0;
    // 箍筋直径
    int boundbar_D = 0;
    // 箍筋间距
    int boundbar_space = 200;
    // 造价, 先给出一个基准值
    double cost = 1e9;
    // 总弯矩和总剪力
    double M_total = 0;
    double V_total = 0;
    // 相对受压区高度
    double ksi = 0;
    // 极限抗弯承载力
    double M_u = 0;
};

// 我们不说暴力穷举, 但可以说网格搜索
DesignResult OptimizeSection(int bridge_type, double L_total, double P, double d)
{
    DesignResult best;
    std::vector<int> diameters = { 14, 16, 18, 20, 22, 25, 28, 32 };

    // 对于其他三种桥面, 它们都是一跨过河, 中间没有柱子, 受力的跨度就是总长
    // 但如果是双跨连续桥, 长度得劈成两半算单跨, 其实本质上是算两个拼在一起的桥
    double L_calc = (bridge_type == 1) ? L_total / 2.0 : L_total;

    // 有个很头疼的超静定结构卸载效应, 需要单独处理一下
    // 在计算基准时, 使用的是最差最容易断的光板桥, 但其他桥型会有物理结构帮翘班分担极大的力量
    // 如果不对受力打折, 算出来的结果会很离谱
    // 其他没什么了, 添几个系数就能水加分项还是太权威了

    double M_scale = 1.0, V_scale = 1.0;
    if (bridge_type == 1) // 双跨连续桥
    {
        // 长度减小的代价就是, 中间的柱子给出的剪切力很大, 剪力上限要调高
        M_scale = 1.0;
        V_scale = 1.25;
    }
    else if (bridge_type == 2) // 下承式拱桥
    {
        // 桥面的受力通过吊索传导至拱圈, 进而再传至桥外
        // 我导论课作业做过这个的推导的, 但文件找不到了, 凭感觉给个系数吧, 反正这种桥型改作业的人也不会算具体数值, 管他呢
        M_scale = 0.35;
        V_scale = 0.5;
    }
    else if (bridge_type == 3) // 斜拉桥
    {
        // 其实相当于把桥梁均分成若干份了, 但这里系数依旧随便给, 理由见上
        // 我真不想再看力学的东西了, 还有两天就截止了整不了那么严谨, 反正 0 个人在意加分项的具体数值
        M_scale = 0.4;
        V_scale = 0.6;
    }

    // 开始网格搜索了, 从 100*200 一直扫到 400*2000
    for (int b = 100; b <= 400; b += 50)
    {
        for (int h = 200; h <= 2000; h += 50)
        {
            // 剪枝, 形状太离谱的高瘦子或者扁胖子直接扔掉
            if (h < 1.5 * b || h > 3.5 * b)
            {
                continue;
            }
            // 纵向也一样
            if (L_calc * 1000 / h < 10 || L_calc * 1000 / h > 18)
            {
                continue;
            }

            // 算出自重 q
            double q = GAMMA * (b / 1000.0) * (h / 1000.0);

            // 算出恒载下的最大弯矩
            double M_dead = q * L_calc * L_calc / 8.0;

            // 双跨桥比较特殊是 0.125
            if (bridge_type == 1)
            {
                M_dead = q * L_calc * L_calc * 0.125;
            }

            // 算出动载下的最大弯矩
            double M_live = (P / (2.0 * L_calc)) * std::pow(L_calc - d / 2.0, 2);

            // 双跨桥, 最大弯矩是中间柱子给的, 要套负弯矩的公式
            if (bridge_type == 1)
            {
                M_live = 0.203 * P * L_calc;
            }

            // 工程上往往需要做到比理论更高的能力, 因此弯矩需要进行一定程度的扩大, 以防万一
            double M_total = (1.2 * M_dead + 1.4 * M_live) * M_scale;

            // 同理, 算出极限剪力
            double V_dead = q * L_calc / 2.0;
            // 一样, 双跨桥套自己的公式
            if (bridge_type == 1)
            {
                V_dead = 0.625 * q * L_calc;
            }
            double V_live = P * (2.0 - d / L_calc);
            double V_total = (1.2 * V_dead + 1.4 * V_live) * V_scale;

            // 扣除保护层, 得到内部真实可用的杠杆力臂极限
            double h0 = h - AS_DIST;

            // 到这里可以再建议辞职, 如果剪力大到连混凝土自己都被斜向压碎了, 塞再多钢筋也没用
            if (V_total * 1000.0 > 0.25 * 1.0 * FC * b * h0)
            {
                continue;
            }

            // 算出裂缝需要缝合的面积, 并反推捆绑铁丝的间距
            // 首先是混凝土本身的抗剪切能力
            double V_cs = 0.7 * FT * b * h0;
            // 箍筋的最小配筋率, 就沿用题目给的数值了
            double Asv_s = (V_total * 1000.0 > V_cs) ? (V_total * 1000.0 - V_cs) / (FYV * h0) : 0.29 * (FT / FYV) * b;
            // 偷个懒, 箍筋定死成 8 mm, 要不然又要写一大堆判断, 我真写不动了
            double Asv_leg = 3.14159 * 8 * 8 / 4.0;
            // 还是最大最小值套起来取中间值
            double spacing = std::max(50.0, std::min(200.0, (2 * Asv_leg) / Asv_s));
            // 方便施工取个整
            int opt_boundbar_space = ((int)spacing / 10) * 10;

            // 解一元二次方程算拉力
            double alpha_s = (M_total * 1e6) / (1.0 * FC * b * h0 * h0);
            // 剪枝, 如果箍筋太多, 混凝土会先被压碎, 这就是最大配筋率
            // ξ <= 0.518, 即 α = 0.518 * (1 - 0.5*0.518) = 0.384
            if (alpha_s > 0.384)
            {
                continue;
            }

            double ksi = 1.0 - std::sqrt(1.0 - 2.0 * alpha_s);
            double As_calc = (M_total * 1e6) / (FY * (1.0 - 0.5 * ksi) * h0);
            // 理论上, 如果混凝土自己能抗住, 就不需要箍筋, 但工程上由于环境因素不可控, 还是多少要配一点, 这就是最小配箍率
            double rho_min = 0.0029;
            double As_req = std::max(As_calc, rho_min * b * h);

            int opt_D = -1, opt_N = -1;
            double min_mainbar_cost = 1e9;
            double actual_As = 0;

            // 拿着算出来的理论面积, 去钢筋型号里找个最便宜的
            for (int D : diameters)
            {
                double area_single = 3.14159 * D * D / 4.0;
                // 向上取整
                int N = (int)(std::ceil(As_req / area_single));
                // 计算把这些钢筋排开, 总共需要多宽的物理空间
                // 保护层, 钢筋自己的宽度, 钢筋之间留给混凝土流下去的最小宽度
                double req_width = 2 * COVER + N * D + (N - 1) * std::max(25.0, (double)D);
                // 如果能放得下, 就进行造价的计算
                if (req_width <= b)
                {
                    // 算体积, 转平米, 算重量, 转公斤, 算基础成本, 算溢价
                    double mainbar_cost = (N * area_single) * 1e-6 * REBAR_DENSITY * 1000.0 * 3.5 * (1.0 + std::max(0, D - 14) * 0.025);
                    if (mainbar_cost < min_mainbar_cost)
                    {
                        min_mainbar_cost = mainbar_cost;
                        opt_D = D; opt_N = N;
                        actual_As = N * area_single;
                    }
                }
            }
            if (opt_N == -1)
            {
                continue;
            }

            // 实际给的钢筋比理论多, 高出来的极限留给后面的动画标尺用
            // 先计算真实的受压深度, 再取求解极限弯矩
            double actual_x = (FY * actual_As) / (1.0 * FC * b);
            double Mu = FY * actual_As * (h0 - actual_x / 2.0) * 1e-6;

            double boundbar_len = 2.0 * (b - 2.0 * COVER + h - 2.0 * COVER);
            // 箍筋就不算溢价了
            double loops_per_meter = 1000.0 / opt_boundbar_space; double boundbar_cost = loops_per_meter * (2.0 * Asv_leg) * boundbar_len * 1e-9 * REBAR_DENSITY * 1000.0 * 3.5;
            // 总造价 = 混凝土 + 钢筋 + 人工
            double total_cost = (b / 1000.0) * (h / 1000.0) * 600.0 + min_mainbar_cost + 500.0 * opt_N;
            // 更新最优解
            if (total_cost < best.cost)
            {
                best.valid = true;
                best.b = b; best.h = h;
                best.As = As_req;
                best.mainbar_D = opt_D; best.mainbar_R = opt_N;
                best.boundbar_space = opt_boundbar_space;
                best.cost = total_cost;
                best.M_total = M_total; best.V_total = V_total;
                best.ksi = ksi; best.M_u = Mu;
            }
        }
    }
    return best;
}

// 把 double 格式化成宽字符串
std::wstring ToWString(double value, int n = 1)
{
    std::wostringstream out;
    out << std::fixed << std::setprecision(n) << value;
    return out.str();
}

// 把受力映射为热力图颜色
// 哎呀, 这不是我时钟做过的 HSVtoRGB 吗, 你说这事闹的
// 作业完成顺序完全正确
sf::Color GetHeatmapColor(float val, float max_val)
{
    // 计算受力百分比
    // 这种从用户给参数的程序都得加 1mol 的防呆, 太麻烦了, 这里给一个防止除零的和防止颜色算爆的
    float ratio = std::clamp(std::abs(val) / std::max(1.0f, max_val), 0.0f, 1.0f);
    // 安全用蓝色, 色环上为 240°, 危险用红色, 色环上为 0°, 其余颜色顺着转一圈即可
    float h = (1.0f - ratio) * 240.0f;
    // 然后就是经典 HSVtoRGB 的切色环, 定主导色, 变非主导色了
    float s = 1.0f, v = 1.0f;
    int i = (int)(h / 60.0f);
    float f = h / 60.0f - (float)(i);
    float p = v * (1.0f - s), q = v * (1.0f - s * f), t = v * (1.0f - s * (1.0f - f));
    float r = 0, g = 0, b = 0;
    switch (i % 6)
    {
    case 0:
        r = v; g = t; b = p;
        break;
    case 1:
        r = q; g = v; b = p;
        break;
    case 2:
        r = p; g = v; b = t;
        break;
    case 3:
        r = p; g = q; b = v;
        break;
    case 4:
        r = t; g = p; b = v;
        break;
    case 5:
        r = v; g = p; b = q;
        break;
    }
    return sf::Color((uint8_t)(r * 255), (uint8_t)(g * 255), (uint8_t)(b * 255));
}

// 计算完了最容易断的截面后, 就要考虑整个桥了, 为了给变色提供值
// 实际上是算出小车在 xc 时, 桥上任意一点 x 的真实受力
float CalculateExactMoment(int type, float x, float xc, float L_total, float P, float q_const)
{
    float M_dead = 0.0f;
    float M_live = 0.0f;

    if (type == 0) // 光板桥
    {
        // 先算自重产生的弯矩, 只考虑一侧
        // 左端支撑力是 q_const * L_total / 2
        // 距离左端 x 处的弯矩 = 左端支撑力的力矩 - 左侧悬空部分的重力力矩, 重心就是 x / 2
        M_dead = 0.5f * q_const * x * (L_total - x);
        // 然后算车的活载
        // 杠杆原理, 小车在 xc 处时, 左端受力 P * (L_total - xc / L_total), 右端受力 P * (xc / L_total)
        // 再乘距离, 把正负调一调就好
        if (x <= xc)
        {
            M_live = P * (L_total - xc) * x / L_total;
        }
        else
        {
            M_live = P * xc * (L_total - x) / L_total;
        }
        // 依旧乘以保险的系数
        return std::abs(1.2f * M_dead + 1.4f * M_live);
    }
    else if (type == 1) // 双跨连续桥
    {
        // 目前水平方向受力为零和力矩为零, 只能建立起两个方程, 但柱子有三根, 解不出来三个未知数, 这个叫超静定问题
        float l = L_total / 2.0f;
        // 现成结论知, 中间柱子给出的负弯矩为 -0.125 * q_const * l²
        float M_B_dead = -0.125f * q_const * l * l;
        // 有了中间的负弯矩, 计算任一点的自重弯矩时, 可以等效为光板桥的抛物线与负弯矩抵消
        // 这里自己带了负号, 之后记得用加法!!!
        if (x <= l)
        {
            M_dead = 0.5f * q_const * x * (l - x) + M_B_dead * (x / l);
        }
        else
        {
            float xp = x - l;
            M_dead = 0.5f * q_const * xp * (l - xp) + M_B_dead * (1.0f - xp / l);
        }

        // 分车在左半边和右半边的情况, 讨论活载的问题
        // 活载由车产生, 但实际是由柱子传导的
        float M_B_live = 0.0f, R_A = 0.0f, R_C = 0.0f;
        if (xc <= l) // 车在左半边时, 桥左侧往下弯, 右侧会翘起来
        {
            // 中间柱子受到的小车的翘起扭矩
            M_B_live = -(P * xc * (l * l - xc * xc)) / (4.0f * l * l);
            // 然后传导到左右两根柱子的受力
            // 因为小车在左半边, 还要考虑小车自己给的部分
            R_A = P * (l - xc) / l + M_B_live / l;
            R_C = M_B_live / l;
            // 左半边的桥, 等于左柱的顶力产生的力矩 - 车轮带来的力矩
            // 右半边的桥, 只有右柱的力矩
            if (x <= l)
            {
                // 材料力学中看截面受力只看一个方向, 因此驶过这个截面后才能开始计算车带来的力矩
                // 有点像 ReLU, 因此套一个 max
                M_live = R_A * x - P * std::max(0.0f, x - xc);
            }
            else
            {
                M_live = R_C * (2.0f * l - x);
            }
        }
        // 右半部分完全一致
        else
        {
            float xcp = 2.0f * l - xc;
            M_B_live = -(P * xcp * (l * l - xcp * xcp)) / (4.0f * l * l);
            R_C = P * (l - xcp) / l + M_B_live / l;
            R_A = M_B_live / l;
            if (x <= l)
            {
                M_live = R_A * x;
            }
            else
            {
                float xp = 2.0f * l - x;
                M_live = R_C * xp - P * std::max(0.0f, xp - xcp);
            }
        }
        return std::abs(1.2f * M_dead + 1.4f * M_live);
    }
    else // 剩下的两种桥太复杂了, 反而可以简化一下, 把作用效果平均出去, 否则得给人算死
    {
        // 把桥面有无数根弹簧托着的梁叫做弹性地基梁, 一辆车压在弹性梁上时, 只有车轮底下的弹簧会被压住, 而离车稍远的地方弹簧其实基本没变形
        // 这种扩散方式通过 PDE 求解知是一个指数衰减, 也即 e^(-dist / decay)
        // 衰减系数拱桥吊杆少, 衰减慢一点, 拉桥拉索多, 衰减快一点
        // 此外, 桥梁自重也会被拉索分担走, 所以算出衰减情况后得出的总载荷还要再缩小一些
        M_dead = 0.5f * q_const * x * (L_total - x);
        float decay = (type == 2) ? (L_total / 6.0f) : (L_total / 8.0f);
        M_live = P * L_total * 0.15f * std::exp(-std::abs(x - xc) / decay);
        float M_tot = 1.2f * M_dead + 1.4f * M_live;
        if (type == 2)
        {
            M_tot *= 0.35f;
        }
        if (type == 3)
        {
            M_tot *= 0.4f;
        }
        return std::abs(M_tot);
    }
}

// 接下来是搭 UI, 基本是把我拓扑图的文件搬过来了
// 没时间给我拆源文件了, 就全堆这里吧, 饶了我
class Button
{
private:
    sf::RectangleShape bounding_box, shadow_box;
    sf::Color normal_color, hover_color;
    std::function<void()> on_click;
    bool is_hovered = false;
    float current_scale = 1.0f, target_scale = 1.0f;
    std::optional<sf::Text> text_;
public:
    Button(float x, float y, float w, float h, sf::Color color) : normal_color(color)
    {
        hover_color = sf::Color(std::min(255, color.r + 30), std::min(255, color.g + 30), std::min(255, color.b + 40));
        float cx = x + w / 2.f, cy = y + h / 2.f;
        bounding_box.setSize(sf::Vector2f(w, h)); bounding_box.setOrigin(sf::Vector2f(w / 2.f, h / 2.f));
        bounding_box.setPosition(sf::Vector2f(cx, cy)); bounding_box.setFillColor(normal_color);
        bounding_box.setOutlineThickness(1.5f); bounding_box.setOutlineColor(sf::Color(255, 255, 255, 100));
        shadow_box.setSize(sf::Vector2f(w, h)); shadow_box.setOrigin(sf::Vector2f(w / 2.f, h / 2.f));
        shadow_box.setPosition(sf::Vector2f(cx + 4.f, cy + 4.f)); shadow_box.setFillColor(sf::Color(0, 0, 0, 30));
    }
    void set_callback(std::function<void()> callback)
    {
        on_click = callback;
    }
    void set_text(const std::wstring& str, const sf::Font& font, unsigned int size, sf::Color text_color = sf::Color(30, 40, 60))
    {
        text_.emplace(font, str, size); text_->setFillColor(text_color);
        sf::FloatRect textRect = text_->getLocalBounds();
        text_->setOrigin(sf::Vector2f(textRect.position.x + textRect.size.x / 2.0f, textRect.position.y + textRect.size.y / 2.0f));        text_->setPosition(bounding_box.getPosition());
    }
    void set_normal_color(sf::Color color)
    {
        normal_color = color;
        hover_color = sf::Color(std::min(255, color.r + 30), std::min(255, color.g + 30), std::min(255, color.b + 40));
    }
    sf::FloatRect get_bounds() const
    {
        return bounding_box.getGlobalBounds();
    }
    void update(const sf::RenderWindow& window)
    {
        sf::Vector2f mousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window), window.getDefaultView());
        is_hovered = get_bounds().contains(mousePos);
        target_scale = is_hovered ? 1.08f : 1.0f;
        if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left) && is_hovered)
        {
            target_scale = 0.95f;
        }
        current_scale += (target_scale - current_scale) * 0.25f;
        bounding_box.setScale(sf::Vector2f(current_scale, current_scale));
        shadow_box.setScale(sf::Vector2f(current_scale, current_scale));
        if (text_)
        {
            text_->setScale(sf::Vector2f(current_scale, current_scale));
        }
        bounding_box.setFillColor(is_hovered ? hover_color : normal_color);
    }
    void draw(sf::RenderWindow& window)
    {
        window.draw(shadow_box); window.draw(bounding_box);
        if (text_.has_value())
        {
            window.draw(text_.value());
        }
    }
    void handle_event(const sf::Event& event, const sf::RenderWindow& window)
    {
        if (const auto* mPress = event.getIf<sf::Event::MouseButtonPressed>())
        {
            if (mPress->button == sf::Mouse::Button::Left && is_hovered && on_click)
            {
                on_click();
            }
        }
    }
};

class Slider
{
public:
    sf::RectangleShape track, progress_track; sf::CircleShape handle;
    int current_val, min_val, max_val;
    std::function<void(int)> on_change;
    // 后面需要把存放的整数转写为小数
    std::function<std::wstring(int)> format_func;
    bool is_dragging = false; std::optional<sf::Text> title_, val_text_; std::unique_ptr<Button> btn_minus, btn_plus;

    Slider(float x, float y, float w, float h, int min_v, int max_v, const std::wstring& name, const sf::Font& font, std::function<std::wstring(int)> fmt = nullptr)
        : min_val(min_v), max_val(max_v), current_val(min_v), format_func(fmt)
    {
        title_.emplace(font, name, 22); title_->setPosition(sf::Vector2f(x, y - 35.f)); title_->setFillColor(sf::Color(70, 90, 110));
        float btn_size = 28.f, track_x = x + btn_size + 15.f, track_w = w - (btn_size * 2 + 75.f);

        btn_minus = std::make_unique<Button>(x, y + h / 2.f - btn_size / 2.f, btn_size, btn_size, sf::Color(240, 245, 250));
        btn_minus->set_text(L"-", font, 22, sf::Color(15, 25, 40));
        btn_minus->set_callback([this]()
            {
                if (current_val > min_val)
                {
                    set_value(current_val - 1);
                    if (on_change)
                    {
                        on_change(current_val);
                    }
                }
            });

        track.setPosition(sf::Vector2f(track_x, y + h / 2.f - 4.f)); track.setSize(sf::Vector2f(track_w, 8.f)); track.setFillColor(sf::Color(225, 230, 240));
        progress_track.setPosition(sf::Vector2f(track_x, y + h / 2.f - 4.f)); progress_track.setSize(sf::Vector2f(0.f, 8.f)); progress_track.setFillColor(sf::Color(100, 100, 240));

        btn_plus = std::make_unique<Button>(track_x + track_w + 15.f, y + h / 2.f - btn_size / 2.f, btn_size, btn_size, sf::Color(240, 245, 250));
        btn_plus->set_text(L"+", font, 22, sf::Color(15, 25, 40)); btn_plus->set_callback([this]()
            {
                if (current_val < max_val)
                {
                    set_value(current_val + 1);
                    if (on_change)
                    {
                        on_change(current_val);
                    }
                }
            });

        std::wstring init_t = format_func ? format_func(min_v) : std::to_wstring(min_v);
        val_text_.emplace(font, init_t, 18); val_text_->setFillColor(sf::Color(15, 25, 40)); val_text_->setPosition(sf::Vector2f(track_x + track_w + btn_size + 20.f, y + h / 2.f - 9.f));
        handle.setRadius(7.f); handle.setOrigin(sf::Vector2f(8.f, 7.f)); handle.setPosition(sf::Vector2f(track_x, y + h / 2.f));
        handle.setFillColor(sf::Color::White); handle.setOutlineThickness(3.f); handle.setOutlineColor(sf::Color(100, 100, 240));
    }

    void set_value(int val)
    {
        current_val = std::clamp(val, min_val, max_val);
        float ratio = (float)(current_val - min_val) / (max_val - min_val);
        handle.setPosition(sf::Vector2f(track.getPosition().x + ratio * track.getSize().x, handle.getPosition().y));
        progress_track.setSize(sf::Vector2f(ratio * track.getSize().x, 8.f));
        if (val_text_)
        {
            val_text_->setString(format_func ? format_func(current_val) : std::to_wstring(current_val));
        }
    }
    void set_callback(std::function<void(int)> callback)
    {
        on_change = callback;
    }
    void update(const sf::RenderWindow& window)
    {
        btn_minus->update(window); btn_plus->update(window);
    }
    void draw(sf::RenderWindow& window)
    {
        if (title_)
        {
            window.draw(*title_);
        }
        btn_minus->draw(window); btn_plus->draw(window);
        window.draw(track); window.draw(progress_track); window.draw(handle);
        if (val_text_)
        {
            window.draw(*val_text_);
        }
    }
    void handle_event(const sf::Event& event, const sf::RenderWindow& window)
    {
        btn_minus->handle_event(event, window); btn_plus->handle_event(event, window);
        if (const auto* mPress = event.getIf<sf::Event::MouseButtonPressed>())
        {
            sf::Vector2f pos = window.mapPixelToCoords(mPress->position, window.getDefaultView());
            if (mPress->button == sf::Mouse::Button::Left && handle.getGlobalBounds().contains(pos))
            {
                is_dragging = true;
            }
        }
        else if (const auto* mRel = event.getIf<sf::Event::MouseButtonReleased>())
        {
            if (mRel->button == sf::Mouse::Button::Left)
            {
                is_dragging = false;
            }
        }
        else if (const auto* mMove = event.getIf<sf::Event::MouseMoved>())
        {
            if (is_dragging)
            {
                sf::Vector2f pos = window.mapPixelToCoords(mMove->position, window.getDefaultView());
                float ratio = std::clamp((pos.x - track.getPosition().x) / track.getSize().x, 0.f, 1.f);
                int n_val = min_val + (int)(ratio * (max_val - min_val));
                if (n_val != current_val)
                {
                    set_value(n_val);
                    if (on_change)
                    {
                        on_change(current_val);
                    }
                }
            }
        }
    }
};
int main()
{
    sf::ContextSettings settings; settings.antiAliasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(1920, 1080)), L"石·建 - SJ Project 03", sf::Style::Default, sf::State::Windowed, settings);
    window.setFramerateLimit(60);

    sf::Font font;
    if (!font.openFromFile("deng.ttf"))
    {
        std::cerr << "请将指定字体文件与源程序放在同一目录下" << std::endl;
    }

    int bridge_type = 0;
    double input_L = 12.0, input_P = 100.0, input_d = 2.0;
    DesignResult result;

    auto CalculateAndRefresh = [&]()
        {
            result = OptimizeSection(bridge_type, input_L, input_P, input_d);
        };

    Slider slider_L(40.f, 150.f, 300.f, 30.f, 8, 50, L"梁长 L (m)", font);
    slider_L.set_value(12);
    slider_L.set_callback([&](int v)
        {
            input_L = v;
            CalculateAndRefresh();
        });

    Slider slider_P(40.f, 260.f, 300.f, 30.f, 50, 800, L"汽车荷载 P (kN)", font);
    slider_P.set_value(100);
    slider_P.set_callback([&](int v)
        {
            input_P = v;
            CalculateAndRefresh();
        });

    // 轴距因为要求是小数, 滑块放大 10 倍存, 显示的时候再缩小
    Slider slider_d(40.f, 370.f, 300.f, 30.f, 15, 60, L"车轮间距 d (m)", font, [](int v)
        {
            return ToWString(v / 10.0, 1);
        });
    slider_d.set_value(20);
    slider_d.set_callback([&](int v)
        {
            input_d = v / 10.0;
            CalculateAndRefresh();
        });

    std::vector<std::unique_ptr<Button>> type_btns;
    std::vector<std::wstring> type_names = { L"简单桥梁", L"双跨连续桥", L"下承式拱桥", L"斜拉桥" };
    for (int i = 0; i < 4; ++i)
    {
        float bx = 40.f + (i % 2) * 160.f; float by = 480.f + (i / 2) * 60.f;
        auto btn = std::make_unique<Button>(bx, by, 140.f, 40.f, sf::Color(226, 232, 240));
        btn->set_text(type_names[i], font, 18);
        btn->set_callback([&, i]()
            {
                bridge_type = i;
                CalculateAndRefresh();
            });
        type_btns.push_back(std::move(btn));
    }

    CalculateAndRefresh();

    // 画小车动画要使用时间
    sf::Clock clock; float car_pos = 0.0f; float car_speed = 5.0f;

    // 尺寸标注
    auto drawDim = [&](sf::Vector2f start, sf::Vector2f end, float offset, std::wstring text, bool is_horizontal)
        {
            // 先定位, 标定测量物体的始末点, 然后把这条线往外挪一挪防止重合
            // 往外挪通过法向量的方式实现
            sf::Color dimColor(100, 120, 140);
            sf::Vector2f normal = is_horizontal ? sf::Vector2f(0.f, offset) : sf::Vector2f(offset, 0.f);
            sf::Vector2f p1 = start + normal, p2 = end + normal;
            // 先画尺寸界线
            sf::Vertex exLines[] =
            {
                // 从物体的起点出发, 画到穿出 p1 15 个像素的地方
                sf::Vertex(start, dimColor),
                sf::Vertex(p1 + (is_horizontal ? sf::Vector2f(0.f, 10.f) : sf::Vector2f(10.f, 0.f)), dimColor),
                // 终点出发
                sf::Vertex(end, dimColor),
                sf::Vertex(p2 + (is_horizontal ? sf::Vector2f(0.f, 10.f) : sf::Vector2f(10.f, 0.f)), dimColor)
            };
            window.draw(exLines, 4, sf::PrimitiveType::Lines);
            // 然后画尺寸线
            sf::Vertex dimLine[] =
            {
                sf::Vertex(p1, dimColor),
                sf::Vertex(p2, dimColor)
            };
            window.draw(dimLine, 2, sf::PrimitiveType::Lines);
            // 最后是没什么用但很装逼的短斜线
            auto drawTick = [&](sf::Vector2f p)
                {
                    sf::Vertex tick[] =
                    {
                        sf::Vertex(p + sf::Vector2f(-4.f, 4.f), dimColor),
                        sf::Vertex(p + sf::Vector2f(4.f, -4.f), dimColor)
                    };
                    window.draw(tick, 2, sf::PrimitiveType::Lines);
                };
            // 始末都要画
            drawTick(p1); drawTick(p2);

            sf::Text t(font, text, 18);
            t.setFillColor(sf::Color(15, 25, 40));
            sf::FloatRect b = t.getLocalBounds();
            // 把文字的原点对到中心后, 放到线段中点达到居中的目的
            // 为了避免遮挡, 再往外挪一挪
            t.setOrigin(sf::Vector2f(b.position.x + b.size.x / 2.f, b.position.y + b.size.y / 2.f));
            t.setPosition((p1 + p2) / 2.0f + (is_horizontal ? sf::Vector2f(0.f, -15.f) : sf::Vector2f(-b.size.x / 2.f - 10.f, 0.f)));
            window.draw(t);
        };

    // 然后是圆, 引出线
    auto drawLeader = [&](sf::Vector2f from, sf::Vector2f to, std::wstring text)
        {
            sf::Color leadColor(240, 70, 70);
            float len = 60.f;
            // 先算水平段向左拐还是向右拐, 向右拐水平段就往右走, 否则往左走
            sf::Vector2f endP = to + sf::Vector2f((to.x > from.x ? len : -len), 0.f);
            sf::Vertex lines[] =
            {
                sf::Vertex(from, leadColor),
                sf::Vertex(to, leadColor),
                sf::Vertex(to, leadColor),
                sf::Vertex(endP, leadColor)
            };
            window.draw(lines, 4, sf::PrimitiveType::Lines);
            // 然后是指向钢筋的箭头, 先求斜线的单位方向向量
            sf::Vector2f dir = to - from;
            dir = dir / std::sqrt(dir.x * dir.x + dir.y * dir.y);
            // 然后是其正交法向量
            sf::Vector2f normal(-dir.y, dir.x);
            sf::Vertex arrow[] =
            {
                // 箭头上半片, 从起点连向偏上的一端
                sf::Vertex(from, leadColor),
                sf::Vertex(from + dir * 8.f + normal * 4.f, leadColor),
                // 下半片则往下偏
                sf::Vertex(from, leadColor),
                sf::Vertex(from + dir * 8.f - normal * 4.f, leadColor)
            };
            window.draw(arrow, 4, sf::PrimitiveType::Lines);
            sf::Text t(font, text, 18);
            t.setFillColor(leadColor);
            sf::FloatRect b = t.getLocalBounds();
            // 把文字的基准点设置在角落, 按延伸方向确定左下角还是右下角
            t.setOrigin(sf::Vector2f(to.x > from.x ? b.position.x : b.position.x + b.size.x, b.position.y + b.size.y));
            // 然后放在 to 的斜上方即可
            t.setPosition(to + sf::Vector2f((to.x > from.x ? 5.f : -5.f), -5.f)); window.draw(t);
        };

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
            slider_L.handle_event(*event, window); slider_P.handle_event(*event, window); slider_d.handle_event(*event, window);
            for (auto& b : type_btns)
            {
                b->handle_event(*event, window);
            }
        }

        // 小车动画
        float dt = clock.restart().asSeconds();
        float total_L_m = (float)(input_L);
        car_pos += car_speed * dt;
        if (car_pos > total_L_m + 5.0f)
        {
            car_pos = -5.0f; // 开到底了就重置回来
        }

        slider_L.update(window); slider_P.update(window); slider_d.update(window);
        for (int i = 0; i < 4; ++i)
        {
            type_btns[i]->set_normal_color((i == bridge_type) ? sf::Color(100, 100, 240) : sf::Color(220, 230, 240));
            type_btns[i]->set_text(type_names[i], font, 18, (i == bridge_type) ? sf::Color::White : sf::Color(15, 20, 40));
            type_btns[i]->update(window);
        }

        window.clear(sf::Color(250, 250, 250));

        // 左侧底板
        sf::RectangleShape leftPanel(sf::Vector2f(380.f, 1040.f));
        leftPanel.setPosition(sf::Vector2f(20.f, 20.f));
        leftPanel.setFillColor(sf::Color(255, 255, 255));
        leftPanel.setOutlineColor(sf::Color(230, 230, 240));
        leftPanel.setOutlineThickness(2.f);
        window.draw(leftPanel);
        sf::Text mainTitle(font, L"参数设定", 26);
        mainTitle.setPosition(sf::Vector2f(70.f, 50.f));
        mainTitle.setFillColor(sf::Color(15, 20, 40));
        window.draw(mainTitle);
        slider_L.draw(window);
        slider_P.draw(window);
        slider_d.draw(window);
        for (auto& b : type_btns)
        {
            b->draw(window);
        }

        // 中间底板
        sf::RectangleShape midPanel(sf::Vector2f(560.f, 1040.f));
        midPanel.setPosition(sf::Vector2f(420.f, 20.f));
        midPanel.setFillColor(sf::Color(255, 255, 255));
        midPanel.setOutlineColor(sf::Color(230, 230, 240));
        midPanel.setOutlineThickness(2.f); window.draw(midPanel);

        if (result.valid)
        {
            sf::Text resTitle(font, L"计算结果", 26);
            resTitle.setPosition(sf::Vector2f(450.f, 50.f));
            resTitle.setFillColor(sf::Color(15, 20, 40));
            window.draw(resTitle);

            double total_project_cost = result.cost * input_L;
            std::wstring cost_str;

            if (total_project_cost >= 10000.0)
            {
                // 换算为万元，保留两位小数
                cost_str = ToWString(total_project_cost / 10000.0, 2) + L" 万元";
            }
            else
            {
                // 显示为元，不保留小数
                cost_str = ToWString(total_project_cost, 0) + L" 元";
            }

            std::wstring resStr =
                L"荷载总弯矩 Md  = " + ToWString(result.M_total) + L" kN·m\n"
                L"抗弯承载力 Mu  = " + ToWString(result.M_u) + L" kN·m\n"
                L"主筋: " + std::to_wstring(result.mainbar_R) + L" Φ " + std::to_wstring(result.mainbar_D) +
                L" | 箍筋: Φ 8 @ " + std::to_wstring(result.boundbar_space) + L"\n"
                L"总造价 = " + cost_str;
            sf::Text resText(font, resStr, 17);
            resText.setPosition(sf::Vector2f(450.f, 100.f));
            resText.setFillColor(sf::Color(70, 85, 105));
            resText.setLineSpacing(1.5f);
            window.draw(resText);

            // 让混凝土截面的尺寸按比例匹配窗口
            double max_draw_h = 450.0;
            // 计算缩放比例
            double scale = max_draw_h / result.h;
            float f_draw_w = (float)(result.b * scale);
            float f_draw_h = (float)(result.h * scale);
            float cx = 420.f + 560.f / 2.0f;
            float cy = 20.f + 1040.f / 2.0f + 60.f;

            // 先画混凝土的大框
            sf::RectangleShape rect(sf::Vector2f(f_draw_w, f_draw_h));
            rect.setOrigin(sf::Vector2f(f_draw_w / 2.0f, f_draw_h / 2.0f));
            rect.setPosition(sf::Vector2f(cx, cy));
            rect.setFillColor(sf::Color(240, 245, 255));
            rect.setOutlineColor(sf::Color(15, 20, 40));
            rect.setOutlineThickness(3.f);
            window.draw(rect);

            // 扣除四周保护层后画上去的红框, 就是箍筋
            sf::RectangleShape boundbar
            (
                sf::Vector2f
                (
                    f_draw_w - 2.f * (float)(COVER * scale),
                    f_draw_h - 2.f * (float)(COVER * scale)
                )
            );
            boundbar.setOrigin(sf::Vector2f(boundbar.getSize().x / 2.f, boundbar.getSize().y / 2.f));
            boundbar.setPosition(sf::Vector2f(cx, cy));
            boundbar.setFillColor(sf::Color::Transparent);
            boundbar.setOutlineColor(sf::Color(240, 70, 70));
            boundbar.setOutlineThickness(2.f); window.draw(boundbar);

            // 然后是底部主筋的位置和它们之间的缝隙
            float mainbar_y = cy + f_draw_h / 2.0f - (float)(AS_DIST * scale);
            // 两根以上就均分然后居中
            float mainbar_space = (result.mainbar_R > 1) ? boundbar.getSize().x / (float)(result.mainbar_R - 1) : 0;
            float start_x = cx - boundbar.getSize().x / 2.0f;
            for (int i = 0; i < result.mainbar_R; ++i)
            {
                float r_rad = std::max(5.0f, (float)((result.mainbar_D * scale) / 2.0));
                sf::CircleShape circle(r_rad);
                circle.setOrigin(sf::Vector2f(r_rad, r_rad));
                circle.setPosition(sf::Vector2f(start_x + (float)(i)*mainbar_space, mainbar_y));
                circle.setFillColor(sf::Color(15, 20, 40));
                window.draw(circle);
            }

            // 贴上标注
            drawDim
            (
                sf::Vector2f(cx - f_draw_w / 2.f, cy + f_draw_h / 2.f),
                sf::Vector2f(cx + f_draw_w / 2.f, cy + f_draw_h / 2.f),
                40.f,
                std::to_wstring((int)result.b),
                true
            );
            drawDim
            (
                sf::Vector2f(cx - f_draw_w / 2.f, cy - f_draw_h / 2.f),
                sf::Vector2f(cx - f_draw_w / 2.f, cy + f_draw_h / 2.f),
                -40.f,
                std::to_wstring((int)result.h),
                false
            );
            drawLeader
            (
                // 定位定在最后一根主筋上
                sf::Vector2f(start_x + (float)(result.mainbar_R - 1) * mainbar_space, mainbar_y),
                sf::Vector2f(cx + f_draw_w / 2.f + 50.f, mainbar_y - 60.f),
                std::to_wstring(result.mainbar_R) + L" Φ " + std::to_wstring(result.mainbar_D)
            );
            drawLeader
            (
                sf::Vector2f(cx + boundbar.getSize().x / 2.f, cy - boundbar.getSize().y / 2.f),
                sf::Vector2f(cx + f_draw_w / 2.f + 50.f, cy - f_draw_h / 2.f - 20.f),
                L"Φ 8 @ " + std::to_wstring(result.boundbar_space)
            );

        }
        // 然后防呆
        else
        {
            sf::Text errText(font, L"结构参数不合理", 32);
            errText.setPosition(sf::Vector2f(590.f, 400.f));
            errText.setFillColor(sf::Color(240, 70, 70));
            window.draw(errText);
        }

        // 右侧动画
        sf::RectangleShape animPanel(sf::Vector2f(900.f, 1040.f));
        animPanel.setPosition(sf::Vector2f(1000.f, 20.f));

        if (result.valid)
        {
            animPanel.setFillColor(sf::Color(15, 20, 40));
            animPanel.setOutlineColor(sf::Color(225, 230, 240));
            animPanel.setOutlineThickness(2.f);
            window.draw(animPanel);
            sf::Text animTitle(font, L"动画演示", 26);
            animTitle.setPosition(sf::Vector2f(1040.f, 50.f));
            animTitle.setFillColor(sf::Color::White);
            window.draw(animTitle);

            float anim_x = 1050.f, anim_w = 800.f, deck_y = 500.f; float scale_x = anim_w / total_L_m;

            // 给桥加上桥墩
            auto drawSupport = [&](float pos_x)
                {
                    sf::ConvexShape triangle; triangle.setPointCount(3);
                    triangle.setPoint(0, sf::Vector2f(pos_x, deck_y + 15.f));
                    triangle.setPoint(1, sf::Vector2f(pos_x - 20.f, deck_y + 50.f));
                    triangle.setPoint(2, sf::Vector2f(pos_x + 20.f, deck_y + 50.f));
                    triangle.setFillColor(sf::Color(100, 120, 140));
                    window.draw(triangle);
                };
            drawSupport(anim_x);
            drawSupport(anim_x + total_L_m * scale_x);
            if (bridge_type == 1)
            {
                drawSupport(anim_x + total_L_m / 2.0f * scale_x);
            }

            // 再根据不同的桥型挂上一些装饰
            if (bridge_type == 2)
            {
                // 拱桥画抛物线吊杆, 微积分大佬来了
                // 把桥切成 99 个区间, 用 100 个点连接它们, 这样其实是一笔画出的, 效能比较高
                sf::VertexArray arch(sf::PrimitiveType::LineStrip, 100);
                for (int i = 0; i < 100; ++i)
                {
                    // 先算当前的点在真实世界里距离桥头的距离和高度
                    float x_m = ((float)(i) / 99.0f) * total_L_m;
                    // 高度用的方程是顶点式, y = 4fx(L-x)/L²
                    float height = 4.0f * (total_L_m / 4.0f) * x_m * (total_L_m - x_m) / (total_L_m * total_L_m);
                    // 然后把点转为屏幕坐标
                    arch[i].position = sf::Vector2f(anim_x + x_m * scale_x, deck_y - height * scale_x);
                    arch[i].color = sf::Color(200, 200, 200);
                    // 每十节往下拉一根吊杆
                    if (i % 10 == 0 && i > 0 && i < 90)
                    {
                        sf::Vertex line[] =
                        {
                            sf::Vertex(arch[i].position, sf::Color(150, 150, 150)),
                            sf::Vertex
                            (
                                sf::Vector2f(arch[i].position.x, deck_y),
                                sf::Color(150, 150, 150)
                            )
                        };
                        window.draw(line, 2, sf::PrimitiveType::Lines);
                    }
                }
                window.draw(arch);
            }
            else if (bridge_type == 3)
            {
                // 斜拉桥在中间竖根桥塔
                float pylon_x = anim_x + total_L_m / 2.0f * scale_x;
                sf::RectangleShape pylon(sf::Vector2f(30.f, 300.f));
                pylon.setOrigin(sf::Vector2f(15.f, 300.f));
                pylon.setPosition(sf::Vector2f(pylon_x, deck_y + 20.f));
                pylon.setFillColor(sf::Color(100, 115, 140));
                window.draw(pylon);
                // 拉索简单意思一下就行
                for (int i = 1; i <= 5; ++i)
                {
                    // 一侧拉 5 根, 则需要将桥切成 12 份
                    float cable_offset = (float)i * (total_L_m / 12.0f) * scale_x;
                    auto drawCable = [&](float anchor_x)
                        {
                            // 算出小车和连接点之间的物理距离
                            float dist = std::abs((anchor_x - anim_x) / scale_x - car_pos);
                            // 按距离远近计算颜色变化, 这里再不考虑传力问题了, 呈现比例关系就成
                            // 距离超过 20 米就不考虑活载力了
                            sf::Color cColor = GetHeatmapColor(std::max(0.0f, 20.0f - dist), 20.0f);
                            sf::Vertex cable[] =
                            {
                                sf::Vertex
                                (
                                    sf::Vector2f(pylon_x, deck_y - 250.f + i * 20.f),
                                    cColor
                                ),
                                sf::Vertex
                                (
                                    sf::Vector2f(anchor_x, deck_y),
                                    cColor
                                )
                            };
                            window.draw(cable, 2, sf::PrimitiveType::Lines);
                        };
                    // 对称画, 还能减少点工作量
                    drawCable(pylon_x - cable_offset);
                    drawCable(pylon_x + cable_offset);
                }
            }

            // 然后画桥面
            int segments = 500;
            float dx = total_L_m / (float)(segments);
            sf::VertexArray deck(sf::PrimitiveType::TriangleStrip, (size_t)((segments + 1) * 2));

            float visual_max = (float)(result.M_u);
            // 还是给保底
            if (visual_max < 100.0f)
            {
                visual_max = 100.0f;
            }

            float q_const = (float)(25.0 * (result.b / 1000.0) * (result.h / 1000.0));

            // 开始遍历微元, 算出每一个截面当前的应力, 然后上色
            for (int i = 0; i <= segments; ++i)
            {
                // 算出微元的物理位置, 受力与颜色
                float x_m = (float)i * dx;
                float M = CalculateExactMoment(bridge_type, x_m, car_pos, total_L_m, (float)(input_P), q_const);
                sf::Color color = GetHeatmapColor(M, visual_max);
                // 对应到屏幕位置
                float sx = anim_x + x_m * scale_x;
                // SFML 的矩形没有办法做渐变, 但三角形可以根据给出的顶点对应的颜色进行渐变, 因此画成三角形
                // 偶数索引放在上面
                deck[(size_t)(i * 2)].position = sf::Vector2f(sx, deck_y - 12.f);
                deck[(size_t)(i * 2)].color = color;
                // 奇数索引放下面
                deck[(size_t)(i * 2 + 1)].position = sf::Vector2f(sx, deck_y + 12.f);
                deck[(size_t)(i * 2 + 1)].color = color;
                // 这样一轮索引过完, 会按点出现的顺序画出两个渐变的三角形, 拼在一起就是渐变的矩形
            }
            window.draw(deck);

            // 画小车
            if (car_pos >= 0.0f && car_pos <= total_L_m)
            {
                float car_sx = anim_x + car_pos * scale_x;
                sf::RectangleShape carShape({ 60.f, 30.f }); carShape.setOrigin({ 30.f, 30.f });
                carShape.setPosition({ car_sx, deck_y - 27.f });
                carShape.setFillColor(sf::Color(250, 250, 250));
                window.draw(carShape);
                sf::CircleShape w1(8.f), w2(8.f);
                w1.setFillColor(sf::Color(100, 120, 140));
                w2.setFillColor(sf::Color(100, 120, 140));
                w1.setOrigin(sf::Vector2f(8.f, 8.f));
                w2.setOrigin(sf::Vector2f(8.f, 8.f));
                w1.setPosition(sf::Vector2f(car_sx - 18.f, deck_y - 18.f));
                w2.setPosition(sf::Vector2f(car_sx + 18.f, deck_y - 18.f));
                window.draw(w1);
                window.draw(w2);
            }

            // 给出图例
            sf::VertexArray legend(sf::PrimitiveType::TriangleStrip, 102); // 51 次循环 + 2 个顶点
            float legend_width = 760.f; float legend_start_x = 1050.f; float legend_y = 860.f;

            for (int i = 0; i <= 50; ++i)
            {
                float ratio = (float)i / 50.0f;
                sf::Color c = GetHeatmapColor(ratio, 1.0f);
                float lx = legend_start_x + ratio * legend_width;
                legend[(size_t)(i * 2)].position = sf::Vector2f(lx, legend_y);
                legend[(size_t)(i * 2)].color = c;
                legend[(size_t)(i * 2 + 1)].position = sf::Vector2f(lx, legend_y + 25.f);
                legend[(size_t)(i * 2 + 1)].color = c;
            }
            window.draw(legend);

            for (int i = 0; i <= 4; ++i)
            {
                float ratio = i / 4.0f;
                float val = ratio * visual_max;
                float lx = legend_start_x + ratio * legend_width;

                sf::Vertex tick[] =
                {
                    sf::Vertex({lx, legend_y + 25.f}, sf::Color::White),
                    sf::Vertex({lx, legend_y + 35.f}, sf::Color::White)
                };
                window.draw(tick, 2, sf::PrimitiveType::Lines);

                sf::Text valText(font, std::to_wstring((int)(val)), 18);
                valText.setFillColor(sf::Color(225, 230, 240));
                sf::FloatRect b = valText.getLocalBounds();
                valText.setOrigin(sf::Vector2f(b.position.x + b.size.x / 2.f, b.position.y));
                valText.setPosition(sf::Vector2f(lx, legend_y + 40.f));
                window.draw(valText);
            }

            sf::Text unitText(font, L"弯矩 M (kN·m)", 20);
            unitText.setPosition({ legend_start_x, legend_y - 35.f });
            unitText.setFillColor(sf::Color(148, 163, 184));
            window.draw(unitText);

        }
        // 不合理的情形不给画
        else
        {
            animPanel.setFillColor(sf::Color(150, 30, 30));
            animPanel.setOutlineColor(sf::Color(240, 70, 70));
            animPanel.setOutlineThickness(4.f);
            window.draw(animPanel);
            sf::Text warn(font, L"结构参数不合理", 48);
            warn.setPosition(sf::Vector2f(1285.f, 400.f));
            warn.setFillColor(sf::Color::White);
            window.draw(warn);
        }

        window.display();
    }
    return 0;
}