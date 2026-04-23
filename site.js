const BASE_BLOCK_LENGTH_MM = 20.0;
const RIGID_SEGMENT_LENGTH_MM = 25.0;
const COMPLIANT_SEGMENT_LENGTH_MM = 15.0;
const JOINT_THICKNESSES_MM = [3.0, 3.25, 3.25, 3.5];
const DEFAULT_MOMENT_ARM_MM = 8.0;
const NOMINAL_JOINT_THICKNESS_MM = 3.25;
const NOMINAL_JOINT_STIFFNESS_NMM_PER_RAD = 50.0;

const ANALYTICAL_COLOR = "#c26a00";
const TWENTY_SPRING_COLOR = "#7b2d8b";
const FEA_COLOR = "#1f6aa5";
const EXPERIMENTAL_COLOR = "#2a8c53";
const DISCREPANCY_COLOR = "#b42318";
const AXIS_COLOR = "#6b5f58";
const GRID_COLOR = "rgba(107, 95, 88, 0.14)";

const FEASTagerows = [
  { cableForceN: 0.0, jointBendsDeg: [0.0, 0.0, 0.0, 0.0] },
  {
    cableForceN: 1.25,
    jointBendsDeg: [-6.643948633467218, -5.841077957025751, -5.817560884381384, -2.1423799974475948],
  },
  {
    cableForceN: 2.5,
    jointBendsDeg: [-6.643948633467218, -5.841077957025751, -5.817560884381384, -2.6483236322109205],
  },
  {
    cableForceN: 3.75,
    jointBendsDeg: [-12.225478272202736, -8.548156775686486, -9.14164324121273, -9.681660442676996],
  },
  {
    cableForceN: 5.0,
    jointBendsDeg: [-24.795716449629683, -18.131323348753224, -19.103509730359235, -17.55832285046337],
  },
  {
    cableForceN: 10.0,
    jointBendsDeg: [-24.795716449629683, -18.131323348753224, -19.103509730359235, -17.55832285046337],
  },
  {
    cableForceN: 20.0,
    jointBendsDeg: [-24.795716449629683, -18.131323348753224, -19.103509730359235, -17.55832285046337],
  },
];

const GRAPH_MOTOR_DELTAS_DEG = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0];
const GRAPH_EXPERIMENTAL_JOINTS_DEG = [
  [0.0, 11.0, 22.0, 31.5, 52.0, 64.0, 77.0, 90.0, 106.0],
  [0.0, 8.5, 15.0, 21.0, 25.0, 31.0, 35.5, 38.5, 42.5],
  [0.0, 9.0, 14.0, 18.0, 27.0, 31.0, 31.0, 35.0, 38.0],
  [0.0, 6.0, 11.0, 14.0, 17.0, 24.0, 25.5, 29.0, 29.0],
];

const VA_LJ = 15e-3;
const VA_H = 6.5e-3;
const VA_BW = 10e-3;
const VA_JH = [2.5e-3, 3.25e-3, 3.25e-3, 3.5e-3];
const VA_E_EFF = 2.5e6;
const VA_RP = 0.015;
const VA_GAIN = 1.0;
const VA_C_TENDON = 0.005;
const VA_PRETENSION = 16.0;
const VA_IJ = VA_JH.map((value) => (1.0 / 12.0) * VA_BW * value ** 3);
const VA_LK = VA_IJ.map((value) => (VA_E_EFF * value) / VA_LJ);
const VA_SUM_FLEX = VA_LK.reduce((sum, value) => sum + 1.0 / value, 0.0);
const VA_DENOM = VA_H ** 2 * VA_SUM_FLEX + VA_C_TENDON;

const SVG_NS = "http://www.w3.org/2000/svg";
const JOINT_NAMES = ["Joint 1", "Joint 2", "Joint 3", "Joint 4"];
const FEA_FORCES = FEASTagerows.map((row) => row.cableForceN);
const MAX_FORCE = FEASTagerows[FEASTagerows.length - 1].cableForceN;

const EXPERIMENTAL_SERIES = graphExperimentalSeries();
const EXPERIMENTAL_FIT = experimentalFitStats(EXPERIMENTAL_SERIES);

const state = {
  force: MAX_FORCE,
  snap: false,
  showFea: true,
  showExperimental: true,
};

const elements = {
  forceSlider: document.getElementById("force-slider"),
  forceInput: document.getElementById("force-input"),
  snapToggle: document.getElementById("snap-toggle"),
  showFeaToggle: document.getElementById("show-fea-toggle"),
  showExpToggle: document.getElementById("show-exp-toggle"),
  stageButtons: document.getElementById("stage-buttons"),
  metricCards: document.getElementById("metric-cards"),
  summaryText: document.getElementById("summary-text"),
  shapePlot: document.getElementById("shape-plot"),
  shapeNote: document.getElementById("shape-note"),
  responsePlot: document.getElementById("response-plot"),
  errorPlot: document.getElementById("error-plot"),
  errorNote: document.getElementById("error-note"),
  comparisonBody: document.getElementById("comparison-body"),
  feaLegends: Array.from(document.querySelectorAll(".fea-legend")),
  expLegends: Array.from(document.querySelectorAll(".exp-legend")),
};

function radians(value) {
  return (value * Math.PI) / 180.0;
}

function degrees(value) {
  return (value * 180.0) / Math.PI;
}

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function lerp(a, b, t) {
  return a + (b - a) * t;
}

function linspace(minimum, maximum, count) {
  if (count <= 1) {
    return [minimum];
  }
  const values = [];
  for (let index = 0; index < count; index += 1) {
    values.push(lerp(minimum, maximum, index / (count - 1)));
  }
  return values;
}

function formatNumber(value, digits = 2) {
  return Number(value).toFixed(digits);
}

function formatSigned(value, digits = 2) {
  return `${value >= 0 ? "+" : ""}${formatNumber(value, digits)}`;
}

function nearestStage(force) {
  return FEA_FORCES.reduce((best, candidate) => (
    Math.abs(candidate - force) < Math.abs(best - force) ? candidate : best
  ), FEA_FORCES[0]);
}

function advance(point, lengthMm, geometryAngleDeg) {
  const angleRad = radians(geometryAngleDeg);
  return [
    point[0] + lengthMm * Math.cos(angleRad),
    point[1] + lengthMm * Math.sin(angleRad),
  ];
}

function displayStateFromJointBends(name, cableForceN, jointBendsDeg) {
  let currentPoint = [BASE_BLOCK_LENGTH_MM, 0.0];
  const centerlinePoints = [[0.0, 0.0], currentPoint];
  const segmentCenters = [];
  const rigidSegments = [];
  const segmentAngles = [];
  let cumulativeAngleDeg = 0.0;

  jointBendsDeg.forEach((jointBendDeg) => {
    const segmentStart = advance(
      currentPoint,
      COMPLIANT_SEGMENT_LENGTH_MM,
      -(cumulativeAngleDeg + 0.5 * jointBendDeg),
    );
    centerlinePoints.push(segmentStart);
    cumulativeAngleDeg += jointBendDeg;
    segmentAngles.push(cumulativeAngleDeg);
    const segmentCenter = advance(segmentStart, 0.5 * RIGID_SEGMENT_LENGTH_MM, -cumulativeAngleDeg);
    const segmentEnd = advance(segmentStart, RIGID_SEGMENT_LENGTH_MM, -cumulativeAngleDeg);
    segmentCenters.push(segmentCenter);
    rigidSegments.push([segmentStart, segmentEnd]);
    centerlinePoints.push(segmentEnd);
    currentPoint = segmentEnd;
  });

  return {
    name,
    cableForceN,
    tipAngleDeg: segmentAngles.length ? segmentAngles[segmentAngles.length - 1] : 0.0,
    jointBendsDeg,
    segmentAnglesDeg: segmentAngles,
    segmentCentersMm: segmentCenters,
    rigidSegmentsMm: rigidSegments,
    centerlinePointsMm: centerlinePoints,
  };
}

function jointTorsionalStiffnesses() {
  return JOINT_THICKNESSES_MM.map((thicknessMm) => (
    NOMINAL_JOINT_STIFFNESS_NMM_PER_RAD * (thicknessMm / NOMINAL_JOINT_THICKNESS_MM) ** 3
  ));
}

function analyticalDisplayState(cableForceN) {
  const stiffnesses = jointTorsionalStiffnesses();
  const jointBendsDeg = stiffnesses.map((stiffness) => (
    -degrees((cableForceN * DEFAULT_MOMENT_ARM_MM) / stiffness)
  ));
  return displayStateFromJointBends("1-Spring PRBM", cableForceN, jointBendsDeg);
}

function solve20SpringState(cableForceN) {
  const jointBendsDeg = JOINT_THICKNESSES_MM.map((thicknessMm) => {
    const wrapRadiusMm = 6.0 - thicknessMm / 2.0;
    const stiffness = 50.0 * (thicknessMm / 3.25) ** 3;
    return -degrees((cableForceN * wrapRadiusMm) / stiffness);
  });
  return displayStateFromJointBends("20-Spring PRBM", cableForceN, jointBendsDeg);
}

function interpolate1D(x, xs, ys) {
  if (x <= xs[0]) {
    return ys[0];
  }
  if (x >= xs[xs.length - 1]) {
    return ys[ys.length - 1];
  }
  for (let index = 0; index < xs.length - 1; index += 1) {
    if (x <= xs[index + 1]) {
      const span = xs[index + 1] - xs[index];
      const t = span === 0 ? 0 : (x - xs[index]) / span;
      return lerp(ys[index], ys[index + 1], t);
    }
  }
  return ys[ys.length - 1];
}

function interpolateFeaState(cableForceN) {
  const clampedForce = clamp(cableForceN, FEA_FORCES[0], FEA_FORCES[FEA_FORCES.length - 1]);
  const jointBendsDeg = [0, 1, 2, 3].map((jointIndex) => (
    interpolate1D(
      clampedForce,
      FEA_FORCES,
      FEASTagerows.map((row) => row.jointBendsDeg[jointIndex]),
    )
  ));
  return displayStateFromJointBends("FEA (CalculiX)", clampedForce, jointBendsDeg);
}

function vaDphi(deltaDeg) {
  return radians(deltaDeg) * VA_GAIN;
}

function vaCableForceFromDelta(deltaDeg) {
  const dphiPre = vaDphi(VA_PRETENSION);
  const dphiTotal = dphiPre + vaDphi(deltaDeg);
  const fcPre = (VA_RP * dphiPre) / VA_DENOM;
  const fcTotal = (VA_RP * dphiTotal) / VA_DENOM;
  return Math.max(0.0, fcTotal - fcPre);
}

function cableLengthMmFromDelta(deltaDeg) {
  return 1000.0 * VA_RP * vaDphi(deltaDeg);
}

function graphExperimentalSeries() {
  const points = GRAPH_MOTOR_DELTAS_DEG.map((deltaDeg, index) => {
    const jointDeltasDeg = GRAPH_EXPERIMENTAL_JOINTS_DEG.map((row) => -row[index]);
    return {
      targetDeltaDeg: deltaDeg,
      cableLengthMm: cableLengthMmFromDelta(deltaDeg),
      cableForceN: vaCableForceFromDelta(deltaDeg),
      jointDeltasDeg,
      tipAngleDeg: jointDeltasDeg.reduce((sum, value) => sum + value, 0.0),
    };
  });
  return points.sort((left, right) => left.cableForceN - right.cableForceN);
}

function closestExperimentalPoint(cableForceN) {
  return EXPERIMENTAL_SERIES.reduce((best, point) => (
    Math.abs(point.cableForceN - cableForceN) < Math.abs(best.cableForceN - cableForceN) ? point : best
  ), EXPERIMENTAL_SERIES[0]);
}

function experimentalFitStats(series) {
  const oneSpringErrors = series.map((point) => analyticalDisplayState(point.cableForceN).tipAngleDeg - point.tipAngleDeg);
  const twentySpringErrors = series.map((point) => solve20SpringState(point.cableForceN).tipAngleDeg - point.tipAngleDeg);

  function stats(label, errors) {
    const absErrors = errors.map((value) => Math.abs(value));
    const mae = absErrors.reduce((sum, value) => sum + value, 0.0) / absErrors.length;
    const rmse = Math.sqrt(errors.reduce((sum, value) => sum + value * value, 0.0) / errors.length);
    const maxAbsErr = Math.max(...absErrors);
    return { label, maeDeg: mae, rmseDeg: rmse, maxAbsErrDeg: maxAbsErr };
  }

  return {
    oneSpring: stats("1-Spring PRBM", oneSpringErrors),
    twentySpring: stats("20-Spring PRBM", twentySpringErrors),
  };
}

function svgNode(name, attributes = {}) {
  const node = document.createElementNS(SVG_NS, name);
  Object.entries(attributes).forEach(([key, value]) => {
    node.setAttribute(key, String(value));
  });
  return node;
}

function drawPlotFrame(svg, width, height, xDomain, yDomain, options) {
  const margin = { left: 58, right: 20, top: 22, bottom: 42 };
  const innerWidth = width - margin.left - margin.right;
  const innerHeight = height - margin.top - margin.bottom;
  let [xMin, xMax] = xDomain;
  let [yMin, yMax] = yDomain;

  if (Math.abs(xMax - xMin) < 1e-9) {
    xMin -= 1.0;
    xMax += 1.0;
  }
  if (Math.abs(yMax - yMin) < 1e-9) {
    yMin -= 1.0;
    yMax += 1.0;
  }

  const scaleX = (value) => margin.left + ((value - xMin) / (xMax - xMin)) * innerWidth;
  const scaleY = (value) => margin.top + (1.0 - (value - yMin) / (yMax - yMin)) * innerHeight;

  const nodes = [
    svgNode("rect", {
      x: margin.left,
      y: margin.top,
      width: innerWidth,
      height: innerHeight,
      rx: 14,
      fill: "rgba(255, 252, 247, 0.78)",
      stroke: "rgba(112, 80, 51, 0.10)",
    }),
  ];

  const xTicks = linspace(xMin, xMax, 6);
  const yTicks = linspace(yMin, yMax, 6);

  xTicks.forEach((tick) => {
    const x = scaleX(tick);
    nodes.push(svgNode("line", {
      x1: x,
      y1: margin.top,
      x2: x,
      y2: height - margin.bottom,
      stroke: GRID_COLOR,
      "stroke-width": 1,
    }));
    nodes.push(svgNode("text", {
      x,
      y: height - 14,
      "text-anchor": "middle",
      "font-size": 12,
      fill: AXIS_COLOR,
    }));
    nodes[nodes.length - 1].textContent = formatNumber(tick, 1);
  });

  yTicks.forEach((tick) => {
    const y = scaleY(tick);
    nodes.push(svgNode("line", {
      x1: margin.left,
      y1: y,
      x2: width - margin.right,
      y2: y,
      stroke: GRID_COLOR,
      "stroke-width": 1,
    }));
    nodes.push(svgNode("text", {
      x: margin.left - 10,
      y: y + 4,
      "text-anchor": "end",
      "font-size": 12,
      fill: AXIS_COLOR,
    }));
    nodes[nodes.length - 1].textContent = formatNumber(tick, 1);
  });

  nodes.push(svgNode("line", {
    x1: margin.left,
    y1: height - margin.bottom,
    x2: width - margin.right,
    y2: height - margin.bottom,
    stroke: AXIS_COLOR,
    "stroke-width": 1.4,
  }));
  nodes.push(svgNode("line", {
    x1: margin.left,
    y1: margin.top,
    x2: margin.left,
    y2: height - margin.bottom,
    stroke: AXIS_COLOR,
    "stroke-width": 1.4,
  }));

  const xLabel = svgNode("text", {
    x: width / 2,
    y: height - 4,
    "text-anchor": "middle",
    "font-size": 13,
    fill: AXIS_COLOR,
    "font-weight": 700,
  });
  xLabel.textContent = options.xLabel;
  nodes.push(xLabel);

  const yLabel = svgNode("text", {
    x: 18,
    y: height / 2,
    "text-anchor": "middle",
    "font-size": 13,
    fill: AXIS_COLOR,
    "font-weight": 700,
    transform: `rotate(-90 18 ${height / 2})`,
  });
  yLabel.textContent = options.yLabel;
  nodes.push(yLabel);

  return { nodes, scaleX, scaleY, margin };
}

function renderShapePlot(analytical, twentySpring, feaState, experimentalState, expPoint) {
  const svg = elements.shapePlot;
  const width = 520;
  const height = 340;
  const statesToDraw = [analytical, twentySpring];
  if (state.showFea && feaState) {
    statesToDraw.push(feaState);
  }
  if (state.showExperimental && experimentalState) {
    statesToDraw.push(experimentalState);
  }

  const allPoints = [[0, 0], [BASE_BLOCK_LENGTH_MM, 0]];
  statesToDraw.forEach((plotState) => {
    plotState.centerlinePointsMm.forEach((point) => allPoints.push(point));
  });

  const xs = allPoints.map((point) => point[0]);
  const ys = allPoints.map((point) => point[1]);
  let xMin = Math.min(...xs) - 8;
  let xMax = Math.max(...xs) + 8;
  let yMin = Math.min(...ys) - 8;
  let yMax = Math.max(...ys) + 8;
  const paddingRatio = (width - 110) / (height - 80);
  const dataWidth = xMax - xMin;
  const dataHeight = Math.max(yMax - yMin, 1);
  const dataRatio = dataWidth / dataHeight;
  if (dataRatio > paddingRatio) {
    const targetHeight = dataWidth / paddingRatio;
    const extra = 0.5 * (targetHeight - dataHeight);
    yMin -= extra;
    yMax += extra;
  } else {
    const targetWidth = dataHeight * paddingRatio;
    const extra = 0.5 * (targetWidth - dataWidth);
    xMin -= extra;
    xMax += extra;
  }

  const frame = drawPlotFrame(svg, width, height, [xMin, xMax], [yMin, yMax], {
    xLabel: "x [mm]",
    yLabel: "z [mm]",
  });
  const nodes = frame.nodes;
  const basePath = [
    `${frame.scaleX(0)},${frame.scaleY(0)}`,
    `${frame.scaleX(BASE_BLOCK_LENGTH_MM)},${frame.scaleY(0)}`,
  ].join(" ");
  nodes.push(svgNode("polyline", {
    points: basePath,
    fill: "none",
    stroke: "#16110f",
    "stroke-width": 8,
    "stroke-linecap": "round",
  }));

  function addState(plotState, color, dashArray) {
    const polylinePoints = plotState.centerlinePointsMm
      .map((point) => `${frame.scaleX(point[0])},${frame.scaleY(point[1])}`)
      .join(" ");
    nodes.push(svgNode("polyline", {
      points: polylinePoints,
      fill: "none",
      stroke: color,
      "stroke-width": 3,
      "stroke-linejoin": "round",
      "stroke-linecap": "round",
      "stroke-dasharray": dashArray || "",
    }));

    plotState.rigidSegmentsMm.forEach((segment) => {
      nodes.push(svgNode("line", {
        x1: frame.scaleX(segment[0][0]),
        y1: frame.scaleY(segment[0][1]),
        x2: frame.scaleX(segment[1][0]),
        y2: frame.scaleY(segment[1][1]),
        stroke: color,
        "stroke-width": 7,
        "stroke-linecap": "round",
        opacity: 0.9,
      }));
    });

    plotState.segmentCentersMm.forEach((point) => {
      nodes.push(svgNode("circle", {
        cx: frame.scaleX(point[0]),
        cy: frame.scaleY(point[1]),
        r: 4.3,
        fill: color,
      }));
    });
  }

  addState(analytical, ANALYTICAL_COLOR, "");
  addState(twentySpring, TWENTY_SPRING_COLOR, "8 6");
  if (state.showFea && feaState) {
    addState(feaState, FEA_COLOR, "3 6");
  }
  if (state.showExperimental && experimentalState) {
    addState(experimentalState, EXPERIMENTAL_COLOR, "10 5 2 5");
  }

  svg.replaceChildren(...nodes);
  if (state.showExperimental && expPoint) {
    elements.shapeNote.textContent = `Nearest experimental point: delta = ${formatNumber(expPoint.targetDeltaDeg, 0)} deg, Fc ~= ${formatNumber(expPoint.cableForceN, 3)} N, tip ~= ${formatNumber(expPoint.tipAngleDeg, 1)} deg`;
  } else {
    elements.shapeNote.textContent = "Experimental overlay is hidden. Use the toggle above to compare against the digitized reference.";
  }
}

function renderResponsePlot(analytical, twentySpring, feaState, expPoint) {
  const svg = elements.responsePlot;
  const width = 520;
  const height = 340;
  const experimentalMaxForce = Math.max(...EXPERIMENTAL_SERIES.map((point) => point.cableForceN));
  const xMax = Math.max(MAX_FORCE, experimentalMaxForce) * 1.05;
  const denseForces = linspace(0.0, xMax, 200);
  const analyticalPoints = denseForces.map((force) => [force, analyticalDisplayState(force).tipAngleDeg]);
  const twentyPoints = denseForces.map((force) => [force, solve20SpringState(force).tipAngleDeg]);
  const feaPoints = denseForces.map((force) => [force, interpolateFeaState(force).tipAngleDeg]);

  const yValues = analyticalPoints.concat(twentyPoints).map((point) => point[1]);
  if (state.showFea) {
    feaPoints.forEach((point) => yValues.push(point[1]));
  }
  if (state.showExperimental) {
    EXPERIMENTAL_SERIES.forEach((point) => yValues.push(point.tipAngleDeg));
  }
  yValues.push(analytical.tipAngleDeg, twentySpring.tipAngleDeg);
  if (state.showFea && feaState) {
    yValues.push(feaState.tipAngleDeg);
  }

  let yMin = Math.min(...yValues);
  let yMax = Math.max(...yValues, 0.0);
  const yPad = Math.max(8.0, 0.08 * Math.max(1.0, yMax - yMin));
  yMin -= yPad;
  yMax += yPad * 0.35;

  const frame = drawPlotFrame(svg, width, height, [0.0, xMax], [yMin, yMax], {
    xLabel: "Cable force [N]",
    yLabel: "Tip angle [deg]",
  });
  const nodes = frame.nodes;

  function addCurve(points, color, dashArray, widthPx) {
    nodes.push(svgNode("polyline", {
      points: points.map((point) => `${frame.scaleX(point[0])},${frame.scaleY(point[1])}`).join(" "),
      fill: "none",
      stroke: color,
      "stroke-width": widthPx,
      "stroke-linejoin": "round",
      "stroke-linecap": "round",
      "stroke-dasharray": dashArray || "",
    }));
  }

  addCurve(analyticalPoints, ANALYTICAL_COLOR, "", 3);
  addCurve(twentyPoints, TWENTY_SPRING_COLOR, "8 6", 3);
  if (state.showFea) {
    addCurve(feaPoints, FEA_COLOR, "3 6", 2.5);
    FEASTagerows.forEach((row) => {
      const feaStage = displayStateFromJointBends("FEA (CalculiX)", row.cableForceN, row.jointBendsDeg);
      nodes.push(svgNode("circle", {
        cx: frame.scaleX(row.cableForceN),
        cy: frame.scaleY(feaStage.tipAngleDeg),
        r: 4.4,
        fill: FEA_COLOR,
      }));
    });
  }

  if (state.showExperimental) {
    addCurve(
      EXPERIMENTAL_SERIES.map((point) => [point.cableForceN, point.tipAngleDeg]),
      EXPERIMENTAL_COLOR,
      "4 6",
      1.6,
    );
    EXPERIMENTAL_SERIES.forEach((point) => {
      nodes.push(svgNode("circle", {
        cx: frame.scaleX(point.cableForceN),
        cy: frame.scaleY(point.tipAngleDeg),
        r: 4.2,
        fill: EXPERIMENTAL_COLOR,
        stroke: "#ffffff",
        "stroke-width": 1.1,
      }));
    });
  }

  [
    { value: analytical.tipAngleDeg, color: ANALYTICAL_COLOR },
    { value: twentySpring.tipAngleDeg, color: TWENTY_SPRING_COLOR },
  ].forEach((marker) => {
    nodes.push(svgNode("circle", {
      cx: frame.scaleX(state.force),
      cy: frame.scaleY(marker.value),
      r: 5.2,
      fill: marker.color,
    }));
  });

  if (state.showFea && feaState) {
    nodes.push(svgNode("circle", {
      cx: frame.scaleX(state.force),
      cy: frame.scaleY(feaState.tipAngleDeg),
      r: 5.2,
      fill: FEA_COLOR,
    }));
  }

  if (state.showExperimental && expPoint) {
    nodes.push(svgNode("circle", {
      cx: frame.scaleX(expPoint.cableForceN),
      cy: frame.scaleY(expPoint.tipAngleDeg),
      r: 5.2,
      fill: EXPERIMENTAL_COLOR,
      stroke: "#ffffff",
      "stroke-width": 1.4,
    }));
  }

  svg.replaceChildren(...nodes);
}

function renderErrorPlot(analytical, twentySpring, feaState, expPoint) {
  const svg = elements.errorPlot;
  const width = 520;
  const height = 320;
  const categories = ["Tip", "J1", "J2", "J3", "J4"];
  const series = [];

  if (state.showExperimental && expPoint) {
    const experimentalState = experimentalDisplayState(expPoint);
    series.push({
      label: "|1-Spring - Exp|",
      color: ANALYTICAL_COLOR,
      values: [
        Math.abs(analytical.tipAngleDeg - experimentalState.tipAngleDeg),
        ...analytical.jointBendsDeg.map((value, index) => Math.abs(value - experimentalState.jointBendsDeg[index])),
      ],
    });
    series.push({
      label: "|20-Spring - Exp|",
      color: TWENTY_SPRING_COLOR,
      values: [
        Math.abs(twentySpring.tipAngleDeg - experimentalState.tipAngleDeg),
        ...twentySpring.jointBendsDeg.map((value, index) => Math.abs(value - experimentalState.jointBendsDeg[index])),
      ],
    });
    if (state.showFea && feaState) {
      series.push({
        label: "|FEA - Exp|",
        color: FEA_COLOR,
        values: [
          Math.abs(feaState.tipAngleDeg - experimentalState.tipAngleDeg),
          ...feaState.jointBendsDeg.map((value, index) => Math.abs(value - experimentalState.jointBendsDeg[index])),
        ],
      });
    }
    elements.errorNote.textContent = `Bars compare the current states against the nearest experimental point at Fc ~= ${formatNumber(expPoint.cableForceN, 3)} N.`;
  } else if (state.showFea && feaState) {
    series.push({
      label: "|1-Spring - FEA|",
      color: ANALYTICAL_COLOR,
      values: [
        Math.abs(analytical.tipAngleDeg - feaState.tipAngleDeg),
        ...analytical.jointBendsDeg.map((value, index) => Math.abs(value - feaState.jointBendsDeg[index])),
      ],
    });
    series.push({
      label: "|20-Spring - FEA|",
      color: TWENTY_SPRING_COLOR,
      values: [
        Math.abs(twentySpring.tipAngleDeg - feaState.tipAngleDeg),
        ...twentySpring.jointBendsDeg.map((value, index) => Math.abs(value - feaState.jointBendsDeg[index])),
      ],
    });
    elements.errorNote.textContent = "Experimental data is hidden, so the bars show each PRBM model against the current FEA state.";
  } else {
    series.push({
      label: "|1-Spring - 20-Spring|",
      color: DISCREPANCY_COLOR,
      values: [
        Math.abs(analytical.tipAngleDeg - twentySpring.tipAngleDeg),
        ...analytical.jointBendsDeg.map((value, index) => Math.abs(value - twentySpring.jointBendsDeg[index])),
      ],
    });
    elements.errorNote.textContent = "Only the two analytical models are active, so the bars show their absolute difference.";
  }

  const margin = { left: 52, right: 18, top: 22, bottom: 46 };
  const innerWidth = width - margin.left - margin.right;
  const innerHeight = height - margin.top - margin.bottom;
  const yMax = Math.max(5.0, ...series.flatMap((entry) => entry.values)) * 1.15;
  const scaleY = (value) => margin.top + (1.0 - value / yMax) * innerHeight;
  const slotWidth = innerWidth / categories.length;
  const barGroupWidth = slotWidth * 0.72;
  const barWidth = barGroupWidth / series.length;
  const nodes = [
    svgNode("rect", {
      x: margin.left,
      y: margin.top,
      width: innerWidth,
      height: innerHeight,
      rx: 14,
      fill: "rgba(255, 252, 247, 0.78)",
      stroke: "rgba(112, 80, 51, 0.10)",
    }),
  ];

  linspace(0, yMax, 5).forEach((tick) => {
    const y = scaleY(tick);
    nodes.push(svgNode("line", {
      x1: margin.left,
      y1: y,
      x2: width - margin.right,
      y2: y,
      stroke: GRID_COLOR,
      "stroke-width": 1,
    }));
    const label = svgNode("text", {
      x: margin.left - 10,
      y: y + 4,
      "text-anchor": "end",
      "font-size": 12,
      fill: AXIS_COLOR,
    });
    label.textContent = formatNumber(tick, 1);
    nodes.push(label);
  });

  categories.forEach((category, categoryIndex) => {
    const xStart = margin.left + slotWidth * categoryIndex + 0.5 * (slotWidth - barGroupWidth);
    const xCenter = margin.left + slotWidth * categoryIndex + slotWidth / 2;
    series.forEach((entry, seriesIndex) => {
      const value = entry.values[categoryIndex];
      const x = xStart + barWidth * seriesIndex;
      const y = scaleY(value);
      nodes.push(svgNode("rect", {
        x,
        y,
        width: Math.max(barWidth - 4, 8),
        height: margin.top + innerHeight - y,
        rx: 6,
        fill: entry.color,
        opacity: 0.88,
      }));
    });
    const xLabel = svgNode("text", {
      x: xCenter,
      y: height - 14,
      "text-anchor": "middle",
      "font-size": 12,
      fill: AXIS_COLOR,
      "font-weight": 700,
    });
    xLabel.textContent = category;
    nodes.push(xLabel);
  });

  nodes.push(svgNode("line", {
    x1: margin.left,
    y1: margin.top,
    x2: margin.left,
    y2: margin.top + innerHeight,
    stroke: AXIS_COLOR,
    "stroke-width": 1.4,
  }));
  nodes.push(svgNode("line", {
    x1: margin.left,
    y1: margin.top + innerHeight,
    x2: width - margin.right,
    y2: margin.top + innerHeight,
    stroke: AXIS_COLOR,
    "stroke-width": 1.4,
  }));

  series.forEach((entry, index) => {
    const y = 24 + index * 18;
    nodes.push(svgNode("line", {
      x1: width - 170,
      y1: y,
      x2: width - 148,
      y2: y,
      stroke: entry.color,
      "stroke-width": 6,
      "stroke-linecap": "round",
    }));
    const label = svgNode("text", {
      x: width - 142,
      y: y + 4,
      "font-size": 12,
      fill: AXIS_COLOR,
    });
    label.textContent = entry.label;
    nodes.push(label);
  });

  svg.replaceChildren(...nodes);
}

function experimentalDisplayState(point) {
  return displayStateFromJointBends("Experimental", point.cableForceN, point.jointDeltasDeg);
}

function renderMetrics(analytical, twentySpring, feaState, expPoint) {
  const cards = [
    {
      label: "Cable force",
      value: `${formatNumber(state.force, 3)} N`,
      note: state.snap ? "Snapped to the nearest FEA stage" : "Continuous force selection",
    },
    {
      label: "1-Spring tip",
      value: `${formatNumber(analytical.tipAngleDeg, 2)} deg`,
      note: "Analytical PRBM baseline",
    },
    {
      label: "20-Spring tip",
      value: `${formatNumber(twentySpring.tipAngleDeg, 2)} deg`,
      note: "Distributed flexure surrogate",
    },
  ];

  if (state.showFea && feaState) {
    cards.push({
      label: "FEA tip",
      value: `${formatNumber(feaState.tipAngleDeg, 2)} deg`,
      note: "Bundled stage interpolation",
    });
  } else if (state.showExperimental && expPoint) {
    cards.push({
      label: "Experimental tip",
      value: `${formatNumber(expPoint.tipAngleDeg, 2)} deg`,
      note: `Nearest point at ${formatNumber(expPoint.cableForceN, 3)} N`,
    });
  } else {
    cards.push({
      label: "Model gap",
      value: `${formatNumber(Math.abs(analytical.tipAngleDeg - twentySpring.tipAngleDeg), 2)} deg`,
      note: "Absolute tip-angle difference",
    });
  }

  elements.metricCards.replaceChildren(
    ...cards.map((card) => {
      const article = document.createElement("article");
      article.className = "metric-card";
      article.innerHTML = `
        <p class="metric-label">${card.label}</p>
        <p class="metric-value">${card.value}</p>
        <p class="metric-note">${card.note}</p>
      `;
      return article;
    }),
  );
}

function renderSummary(analytical, twentySpring, feaState, expPoint) {
  const fitWinner = EXPERIMENTAL_FIT.oneSpring.rmseDeg <= EXPERIMENTAL_FIT.twentySpring.rmseDeg
    ? EXPERIMENTAL_FIT.oneSpring
    : EXPERIMENTAL_FIT.twentySpring;
  const items = [
    `Current cable force: ${formatNumber(state.force, 3)} N`,
    `Tip angles: 1-Spring ${formatNumber(analytical.tipAngleDeg, 2)} deg, 20-Spring ${formatNumber(twentySpring.tipAngleDeg, 2)} deg`,
  ];

  if (state.showFea && feaState) {
    items.push(
      `FEA tip: ${formatNumber(feaState.tipAngleDeg, 2)} deg, with 1-Spring minus FEA = ${formatSigned(analytical.tipAngleDeg - feaState.tipAngleDeg, 2)} deg`,
    );
  }

  if (state.showExperimental && expPoint) {
    items.push(
      `Nearest experimental point: delta = ${formatNumber(expPoint.targetDeltaDeg, 0)} deg, Fc ~= ${formatNumber(expPoint.cableForceN, 3)} N, tip = ${formatNumber(expPoint.tipAngleDeg, 2)} deg`,
    );
    items.push(
      `Experimental errors: 1-Spring ${formatSigned(analytical.tipAngleDeg - expPoint.tipAngleDeg, 2)} deg, 20-Spring ${formatSigned(twentySpring.tipAngleDeg - expPoint.tipAngleDeg, 2)} deg`,
    );
    if (state.showFea && feaState) {
      items.push(
        `FEA minus experimental: ${formatSigned(feaState.tipAngleDeg - expPoint.tipAngleDeg, 2)} deg`,
      );
    }
  }

  items.push(
    `Overall fit to the digitized experimental series: ${fitWinner.label} is closer. RMSE = ${formatNumber(fitWinner.rmseDeg, 2)} deg, MAE = ${formatNumber(fitWinner.maeDeg, 2)} deg, max abs error = ${formatNumber(fitWinner.maxAbsErrDeg, 2)} deg.`,
  );

  const list = document.createElement("ul");
  items.forEach((item) => {
    const li = document.createElement("li");
    li.textContent = item;
    list.appendChild(li);
  });
  elements.summaryText.replaceChildren(list);
}

function renderTable(analytical, twentySpring, feaState, expPoint) {
  const experimentalState = state.showExperimental && expPoint ? experimentalDisplayState(expPoint) : null;
  const rows = [
    {
      label: "Tip angle",
      oneSpring: analytical.tipAngleDeg,
      twentySpring: twentySpring.tipAngleDeg,
      fea: feaState ? feaState.tipAngleDeg : null,
      experimental: experimentalState ? experimentalState.tipAngleDeg : null,
    },
    ...JOINT_NAMES.map((name, index) => ({
      label: name,
      oneSpring: analytical.jointBendsDeg[index],
      twentySpring: twentySpring.jointBendsDeg[index],
      fea: feaState ? feaState.jointBendsDeg[index] : null,
      experimental: experimentalState ? experimentalState.jointBendsDeg[index] : null,
    })),
  ];

  elements.comparisonBody.replaceChildren(
    ...rows.map((row) => {
      const tr = document.createElement("tr");
      const expValue = row.experimental;
      tr.innerHTML = `
        <td>${row.label}</td>
        <td>${formatNumber(row.oneSpring, 2)}</td>
        <td>${formatNumber(row.twentySpring, 2)}</td>
        <td>${row.fea == null ? "--" : formatNumber(row.fea, 2)}</td>
        <td>${expValue == null ? "--" : formatNumber(expValue, 2)}</td>
        <td>${expValue == null ? "--" : formatSigned(row.oneSpring - expValue, 2)}</td>
        <td>${expValue == null ? "--" : formatSigned(row.twentySpring - expValue, 2)}</td>
      `;
      return tr;
    }),
  );
}

function renderStageButtons() {
  elements.stageButtons.replaceChildren(
    ...FEA_FORCES.map((force) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "stage-chip";
      button.textContent = `${formatNumber(force, 2)} N`;
      button.addEventListener("click", () => setForce(force));
      return button;
    }),
  );
}

function syncControls() {
  elements.forceSlider.value = String(state.force);
  elements.forceInput.value = formatNumber(state.force, 3);
  elements.snapToggle.checked = state.snap;
  elements.showFeaToggle.checked = state.showFea;
  elements.showExpToggle.checked = state.showExperimental;
  Array.from(elements.stageButtons.children).forEach((button, index) => {
    button.classList.toggle("active", Math.abs(FEA_FORCES[index] - state.force) < 1e-6);
  });
  elements.feaLegends.forEach((node) => node.classList.toggle("hidden", !state.showFea));
  elements.expLegends.forEach((node) => node.classList.toggle("hidden", !state.showExperimental));
}

function render() {
  const analytical = analyticalDisplayState(state.force);
  const twentySpring = solve20SpringState(state.force);
  const feaState = state.showFea ? interpolateFeaState(state.force) : null;
  const expPoint = state.showExperimental ? closestExperimentalPoint(state.force) : null;
  const experimentalState = state.showExperimental && expPoint ? experimentalDisplayState(expPoint) : null;

  syncControls();
  renderMetrics(analytical, twentySpring, feaState, expPoint);
  renderSummary(analytical, twentySpring, feaState, expPoint);
  renderShapePlot(analytical, twentySpring, feaState, experimentalState, expPoint);
  renderResponsePlot(analytical, twentySpring, feaState, expPoint);
  renderErrorPlot(analytical, twentySpring, feaState, expPoint);
  renderTable(analytical, twentySpring, feaState, expPoint);
}

function setForce(nextForce) {
  let force = clamp(Number(nextForce), 0.0, MAX_FORCE);
  if (state.snap) {
    force = nearestStage(force);
  }
  state.force = force;
  render();
}

function bindEvents() {
  elements.forceSlider.max = String(MAX_FORCE);
  elements.forceInput.max = String(MAX_FORCE);

  elements.forceSlider.addEventListener("input", (event) => {
    setForce(event.target.value);
  });
  elements.forceInput.addEventListener("change", (event) => {
    setForce(event.target.value);
  });
  elements.snapToggle.addEventListener("change", (event) => {
    state.snap = event.target.checked;
    setForce(state.force);
  });
  elements.showFeaToggle.addEventListener("change", (event) => {
    state.showFea = event.target.checked;
    render();
  });
  elements.showExpToggle.addEventListener("change", (event) => {
    state.showExperimental = event.target.checked;
    render();
  });
}

renderStageButtons();
bindEvents();
render();
