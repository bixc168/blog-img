import { launch } from 'puppeteer';
import { writeFileSync } from 'fs';
import * as math from 'mathjs';

class BezierTrajectory {
    _bztsg(dataTrajectory) {
        const lengthOfdata = dataTrajectory.length;

        function staer(x) {
            const t = (x - dataTrajectory[0][0]) / (dataTrajectory[dataTrajectory.length - 1][0] - dataTrajectory[0][0]);
            let y = [0, 0];

            for (let s = 0; s < dataTrajectory.length; s++) {
                const coefficient = math.factorial(lengthOfdata - 1) /
                    (math.factorial(s) * math.factorial(lengthOfdata - 1 - s));

                y[0] += dataTrajectory[s][0] * (coefficient * Math.pow(t, s) * Math.pow((1 - t), lengthOfdata - 1 - s));
                y[1] += dataTrajectory[s][1] * (coefficient * Math.pow(t, s) * Math.pow((1 - t), lengthOfdata - 1 - s));
            }

            return y[1];
        }

        return staer;
    }

    _type(type, x, numberList) {
        let numberListre = [];
        const pin = (x[1] - x[0]) / numberList;

        if (type === 0) {
            numberListre = Array.from({ length: numberList }, (_, i) => i * pin);
            if (pin >= 0) {
                numberListre = numberListre.reverse();
            }
        } else if (type === 1) {
            numberListre = Array.from({ length: numberList }, (_, i) => 1 * Math.pow(i * pin, 2));
            numberListre = numberListre.reverse();
        } else if (type === 2) {
            numberListre = Array.from({ length: numberList }, (_, i) => 1 * Math.pow(i * pin - x[1], 2));
        } else if (type === 3) {
            const dataTrajectory = [
                [0, 0],
                [(x[1] - x[0]) * 0.8, (x[1] - x[0]) * 0.6],
                [x[1] - x[0], 0]
            ];
            const fun = this._bztsg(dataTrajectory);
            numberListre = [0];
            for (let i = 1; i < numberList; i++) {
                numberListre.push(fun(i * pin) + numberListre[numberListre.length - 1]);
            }
            if (pin >= 0) {
                numberListre = numberListre.reverse();
            }
        }

        // 添加对 NaN 的检查
        numberListre = numberListre.map(value => Math.abs(value - Math.max(...numberListre)));

        if (numberListre.some(isNaN)) {
            console.log("Warning: numberListre contains NaN values");
            return null;
        }

        const minIndex = numberListre.indexOf(Math.min(...numberListre));
        const maxIndex = numberListre.indexOf(Math.max(...numberListre));

        const adjustedListre = numberListre.map(value => value - numberListre[minIndex]);

        // Ensure that the denominator is not 0 to avoid division by zero
        const denominator = numberListre[maxIndex] - numberListre[minIndex];
        const normalizedArray = denominator !== 0 ? adjustedListre.map(value => (value / denominator) * (x[1] - x[0]) + x[0] ) : adjustedListre;

        // console.log(normalizedArray);

        normalizedArray[0] = x[0];
        normalizedArray[normalizedArray.length - 1] = x[1];
        return normalizedArray;
    }

    getFun(s) {
        const dataTrajectory = s.map(i => Array.from(i));
        return this._bztsg(dataTrajectory);
    }

    /**
     * 模拟人手动滑动的轨迹生成器
     * @param {number[]} start - 开始点的坐标，如 [0, 0]
     * @param {number[]} end - 结束点的坐标，如 [100, 100]
     * @param {number} [le=1] - 几阶贝塞尔曲线，越大越复杂，默认为 1
     * @param {number} [deviation=0] - 轨迹上下波动的范围，默认为 0
     * @param {number} [bias=0.5] - 波动范围的分布位置，默认为 0.5
     * @returns {Object} 返回一个对象，包含属性 equation 对应该曲线的方程，P 对应贝塞尔曲线的影响点
     */
    simulation(start, end, le = 1, deviation = 0, bias = 0.5) {
        start = Array.from(start);
        end = Array.from(end);
        let cbb = [];

        if (le !== 1) {
            const e = (1 - bias) / (le - 1);
            cbb = [...Array(le - 1)].map((_, i) => [bias + e * i, bias + e * (i + 1)]);
        }

        const dataTrajectoryList = [start];

        let t = Math.random() > 0.5 ? 1 : -1;
        let w = 0;

        for (const i of cbb) {
            const px1 = start[0] + (end[0] - start[0]) * (Math.random() * (i[1] - i[0]) + (i[0]));
            const p = [px1, this._bztsg([start, end])(px1) + t * deviation];
            dataTrajectoryList.push(p);
            w += 1;

            if (w >= 2) {
                w = 0;
                t = -1 * t;
            }
        }

        dataTrajectoryList.push(end);
        return { equation: this._bztsg(dataTrajectoryList), P: dataTrajectoryList };
    }

    /**
     * 生成贝塞尔曲线轨迹数组和影响点。
     * @param {Array} start - 开始点的坐标，如 start = [0, 0]
     * @param {Array} end - 结束点的坐标，如 end = [100, 100]
     * @param {number} numberList - 返回的数组的轨迹点的数量，numberList = 150
     * @param {number} le - 几阶贝塞尔曲线，越大越复杂，如 le = 4
     * @param {number} deviation - 轨迹上下波动的范围，如 deviation = 10
     * @param {number} bias - 波动范围的分布位置，如 bias = 0.5
     * @param {number} type - 0表示均速滑动，1表示先慢后快，2表示先快后慢，3表示先慢中间快后慢，如 type = 1
     * @param {number} cbb - 在终点来回摆动的次数
     * @param {number} yhh - 在终点来回摆动的范围
     * @returns {Object} - 返回一个字典，trackArray对应轨迹数组，P对应贝塞尔曲线的影响点
     */
    trackArray(start, end, numberList, le = 4, deviation = 25, bias = 0.5, type = 2, cbb = 0, yhh = 0) {
        let s = [];
        const fun = this.simulation(start, end, le, deviation, bias);
        let w = fun.P;
        const equation = fun.equation;

        if (cbb !== 0) {
            let numberListOfcbb = Math.round(numberList * 0.2 / (cbb + 1));
            numberList -= (numberListOfcbb * (cbb + 1));

            const xTrackArray = this._type(type, [start[0], end[0]], numberList);
            for (const i of xTrackArray) {
                s.push([i, equation(i)]);
            }

            const dq = yhh / cbb;
            let kg = 0;
            let ends = [...end];

            for (let i = 0; i < cbb; i++) {
                const d = kg === 0 ?
                    [end[0] + (yhh - dq * i), ((end[1] - start[1]) / (end[0] - start[0])) * (end[0] + (yhh - dq * i)) + (end[1] - ((end[1] - start[1]) / (end[0] - start[0])) * end[0])] :
                    [end[0] - (yhh - dq * i), ((end[1] - start[1]) / (end[0] - start[0])) * (end[0] - (yhh - dq * i)) + (end[1] - ((end[1] - start[1]) / (end[0] - start[0])) * end[0])];

                // console.log(d);
                const y = this.trackArray(ends, d, numberListOfcbb, le = 2, deviation = 0, bias = 0.5, type = 0, cbb = 0, yhh = 10);
                s = s.concat(y.trackArray);
                ends = d;
            }

            const y = this.trackArray(ends, end, numberListOfcbb, le = 2, deviation = 0, bias = 0.5, type = 0, cbb = 0, yhh = 10);
            s = s.concat(y.trackArray);
        } else {
            const xTrackArray = this._type(type, [start[0], end[0]], numberList);
            for (const i of xTrackArray) {
                s.push([i, equation(i)]);
            }
        }

        // console.log(s);
        return { trackArray: s, P: w };
    }
}
export default BezierTrajectory;


// // 定义开始点和结束点
// const start_point = [10, 10]
// const end_point = [100, 200]

// const bz = new BezierTrajectory();

// const result = bz.simulation(start_point, end_point, 5, 5, 0.5)

// // 获取生成的插值函数和影响点
// const equation = result.equation;
// const P_points = result.P;

// // 使用插值函数计算轨迹上的点
// const x_values = Array.from({ length: 50 }, (_, index) => start_point[0] + (end_point[0] - start_point[0]) * index / 99);
// const y_values = x_values.map(x => equation(x));

// // 打印结果
// console.log("Equation:", equation);
// console.log("P points:", P_points);
// console.log("Generated track:", x_values.map((x, index) => [x, y_values[index]]));

// const result2 = bz.trackArray(start_point, end_point, 59)

// // 获取生成的插值函数和影响点
// const equation2 = result2.trackArray;
// const P_points2 = result2.P;
// for (const [x, y] of equation2.entries()) {
//     console.log("x:", x,"y:",y);
// }

// // 打印结果
// console.log("P points:", P_points2);
