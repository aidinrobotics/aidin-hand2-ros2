import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from './vendor/URDFLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import {
  THUMB_Q4_RAD, LONG_Q4_RAD, LONG_Q3_FLOOR_RAD,
  THUMB_ABDUCTION_MAX_DEG, LONG_ABDUCTION_MAX_DEG,
  THUMB_FLEXION_MAX_DEG, LONG_FLEXION_MAX_DEG,
} from './handTables.js';

const RAD2DEG = 180 / Math.PI;
const HAND_SEPARATION_M = 0.15;
const AXES_LENGTH_M = 0.03;

const ISO_VIEW = new THREE.Vector3(0.55, 0.38, 0.35).normalize();
const WORLD_UP = new THREE.Vector3(0, 1, 0);
const FIT_MARGIN = 1.08;

const LINK_LIFT = 0.15;

const TACTILE_GREY = 0.40;

const GROUPS = [
  { label: 'Thumb', prefix: 'thumb', joints: [0, 1, 2, 3] },
  { label: 'Index', prefix: 'index', joints: [1, 2, 3] },
  { label: 'Middle', prefix: 'middle', joints: [1, 2, 3] },
  { label: 'Ring', prefix: 'ring', joints: [1, 2, 3] },
  { label: 'Baby', prefix: 'baby', joints: [1, 2, 3] },
];

const STEPS_DEG = [0.1, 1, 10];
const DEFAULT_STEP_DEG = 1;

const ROTATE_ICON =
  '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.6"'
  + ' stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">'
  + '<path d="M20 11a8 8 0 1 0-2.3 6"/><path d="M20 4.5V11h-6.2"/></svg>';

function sampleAtDegrees(table, degrees) {
  if (!Number.isFinite(degrees) || degrees <= 0) return table[0];
  const last = table.length - 1;
  if (degrees >= last) return table[last];
  const i = Math.floor(degrees);
  return table[i] + (table[i + 1] - table[i]) * (degrees - i);
}

function passiveDistalRad(isThumb, q3Rad) {
  const table = isThumb ? THUMB_Q4_RAD : LONG_Q4_RAD;
  return sampleAtDegrees(table, q3Rad * RAD2DEG);
}

function longDistalFloorRad(abductionRad, flexionRad) {
  const rows = LONG_Q3_FLOOR_RAD;
  const columns = rows[0].length;
  const clampIndex = (value, limit) => Math.max(0, Math.min(limit, value));

  const flexion = clampIndex(flexionRad * RAD2DEG, rows.length - 1);
  const abduction = clampIndex(Math.abs(abductionRad) * RAD2DEG, columns - 1);
  const f0 = Math.floor(flexion);
  const a0 = Math.floor(abduction);
  const f1 = Math.min(f0 + 1, rows.length - 1);
  const a1 = Math.min(a0 + 1, columns - 1);
  const tf = flexion - f0;
  const ta = abduction - a0;

  const low = rows[f0][a0] + (rows[f0][a1] - rows[f0][a0]) * ta;
  const high = rows[f1][a0] + (rows[f1][a1] - rows[f1][a0]) * ta;
  return low + (high - low) * tf;
}

function boundaryOf(isThumb) {
  const table = isThumb ? THUMB_ABDUCTION_MAX_DEG : LONG_ABDUCTION_MAX_DEG;
  const flexionMax = isThumb ? THUMB_FLEXION_MAX_DEG : LONG_FLEXION_MAX_DEG;
  const points = table.map((abduction, flexion) => [flexion, abduction]);
  if (points[points.length - 1][0] < flexionMax) points.push([flexionMax, 0]);
  return points;
}

function abductionRoomRad(isThumb, flexionRad) {
  const table = isThumb ? THUMB_ABDUCTION_MAX_DEG : LONG_ABDUCTION_MAX_DEG;
  return sampleAtDegrees(table, flexionRad * RAD2DEG) / RAD2DEG;
}

function flexionRangeRad(isThumb, abductionRad) {
  const table = isThumb ? THUMB_ABDUCTION_MAX_DEG : LONG_ABDUCTION_MAX_DEG;
  const flexionMax = isThumb ? THUMB_FLEXION_MAX_DEG : LONG_FLEXION_MAX_DEG;
  const want = Math.abs(abductionRad) * RAD2DEG;

  let low = 0;
  while (low < table.length - 1 && table[low] < want) low += 1;
  let high = table.length - 1;
  while (high > 0 && table[high] < want) high -= 1;
  if (low > high) return { lower: 0, upper: 0 };

  const cross = (i, j) => (table[j] === table[i] ? i : i + (want - table[i]) / (table[j] - table[i]));
  const lowerDeg = low > 0 ? cross(low - 1, low) : 0;
  const upperDeg = high < table.length - 1 ? cross(high, high + 1) : flexionMax;
  return { lower: lowerDeg / RAD2DEG, upper: Math.min(upperDeg, flexionMax) / RAD2DEG };
}

function projectIntoWorkspace(isThumb, flexionRad, abductionRad) {
  const flexionMax = isThumb ? THUMB_FLEXION_MAX_DEG : LONG_FLEXION_MAX_DEG;
  const table = isThumb ? THUMB_ABDUCTION_MAX_DEG : LONG_ABDUCTION_MAX_DEG;

  const flexion = flexionRad * RAD2DEG;
  const sign = abductionRad < 0 ? -1 : 1;
  const abduction = Math.abs(abductionRad) * RAD2DEG;

  if (flexion >= 0 && flexion <= flexionMax
      && abduction <= sampleAtDegrees(table, flexion) + 1e-9) {
    return { flexionRad, abductionRad };
  }

  const points = boundaryOf(isThumb);
  let best = null;
  for (let i = 0; i + 1 < points.length; i += 1) {
    const [x0, y0] = points[i];
    const [x1, y1] = points[i + 1];
    const dx = x1 - x0;
    const dy = y1 - y0;
    const lengthSq = dx * dx + dy * dy;
    const t = lengthSq === 0 ? 0
      : Math.max(0, Math.min(1, ((flexion - x0) * dx + (abduction - y0) * dy) / lengthSq));
    const px = x0 + t * dx;
    const py = y0 + t * dy;
    const distanceSq = (px - flexion) ** 2 + (py - abduction) ** 2;
    if (best === null || distanceSq < best.distanceSq) best = { distanceSq, px, py };
  }
  return { flexionRad: best.px / RAD2DEG, abductionRad: sign * best.py / RAD2DEG };
}

function dressMaterials(robot) {
  const pads = new Set();
  for (const [name, link] of Object.entries(robot.links)) {
    if (!name.includes('tactile')) continue;
    link.traverse((node) => {
      for (const material of [node.material].flat()) if (material) pads.add(material);
    });
  }

  const done = new Set();
  robot.traverse((node) => {
    for (const material of [node.material].flat()) {
      if (!material || !material.color || done.has(material)) continue;
      done.add(material);
      if (pads.has(material)) {
        material.color.setRGB(TACTILE_GREY, TACTILE_GREY, TACTILE_GREY);
      } else {
        const { r, g, b } = material.color;
        const lift = (c) => c + (1 - c) * LINK_LIFT;
        material.color.setRGB(lift(r), lift(g), lift(b));
      }
      material.color.convertSRGBToLinear();
      material.specular.setRGB(0, 0, 0);
      material.shininess = 0;
    }
  });
}

function loadHand(side) {
  return new Promise((resolve, reject) => {
    const loader = new URDFLoader();
    const stl = new STLLoader();
    loader.packages = { viewer_model: './model' };

    let pending = 0;
    let robot = null;
    const settle = () => {
      if (!robot || pending !== 0) return;
      dressMaterials(robot);
      resolve(robot);
    };

    loader.loadMeshCb = (path, _manager, material, done) => {
      pending += 1;
      stl.load(
        path,
        (geometry) => {

          done(new THREE.Mesh(geometry, material || new THREE.MeshPhongMaterial()));
          pending -= 1;
          settle();
        },
        undefined,
        (error) => {
          pending -= 1;
          reject(new Error(`${path}: ${error.message ?? error}`));
        });
    };

    loader.load(`./model/${side}.urdf`, (loaded) => { robot = loaded; settle(); },
                undefined, reject);
  });
}

function movableJoints(robot, side) {
  const out = new Map();
  for (const [name, joint] of Object.entries(robot.joints)) {
    if (joint.jointType === 'revolute') out.set(name.slice(side.length + 1), joint);
  }
  return out;
}

class Hand {
  constructor(side, robot, panel) {
    this.side = side;
    this.robot = robot;
    this.joints = movableJoints(robot, side);
    this.sliders = new Map();
    this.valueLabels = new Map();

    this.reachLimit = new Map();
    this.stepDeg = DEFAULT_STEP_DEG;
    this.buildPanel(panel);
    this.apply();
  }

  jointOf(suffix) {
    return this.joints.get(suffix);
  }

  buildPanel(panel) {
    panel.replaceChildren();

    const title = document.createElement('h2');
    const name = document.createElement('span');
    name.className = 'name';
    name.textContent = this.side === 'left' ? 'Left hand' : 'Right hand';
    title.append(name, this.buildJogPicker());

    const zeroAll = document.createElement('button');
    zeroAll.title = '이 손의 모든 관절을 0으로';
    zeroAll.innerHTML = ROTATE_ICON;
    zeroAll.addEventListener('click', () => {
      for (const slider of this.sliders.values()) slider.value = '0';
      this.apply();
    });
    title.append(zeroAll);
    panel.append(title);

    for (const group of GROUPS) {
      const section = document.createElement('section');
      section.className = 'group';

      const heading = document.createElement('h3');
      heading.textContent = group.label;
      section.append(heading);

      for (const index of group.joints) {
        section.append(this.buildSlider(`${group.prefix}_joint${index}`));
      }

      section.append(this.buildPassiveRow(`${group.prefix}_joint4`));
      panel.append(section);
    }
  }

  buildJogPicker() {
    const jog = document.createElement('div');
    jog.className = 'jog';

    const label = document.createElement('span');
    label.className = 'jog-label';
    label.textContent = 'jog';

    const segments = document.createElement('div');
    segments.className = 'seg';
    segments.role = 'radiogroup';
    segments.title = '\u2212 와 + 한 번이 움직이는 각도';

    const buttons = STEPS_DEG.map((deg) => {
      const button = document.createElement('button');
      button.type = 'button';
      button.textContent = `${deg}\u00b0`;
      button.addEventListener('click', () => { this.stepDeg = deg; mark(); });
      segments.append(button);
      return { deg, button };
    });

    const mark = () => {
      for (const { deg, button } of buttons) {
        button.setAttribute('aria-checked', String(deg === this.stepDeg));
      }
    };

    mark();
    jog.append(label, segments);
    return jog;
  }

  buildSlider(suffix) {
    const joint = this.jointOf(suffix);
    const row = document.createElement('div');
    row.className = 'jogrow';

    const name = document.createElement('span');
    name.className = 'jr-label';
    name.textContent = suffix.replace(/^[a-z]+_/, '');

    const slider = document.createElement('input');
    slider.className = 'jr-slider';
    slider.type = 'range';
    slider.min = String(joint.limit.lower);
    slider.max = String(joint.limit.upper);

    slider.step = 'any';
    slider.value = '0';
    slider.addEventListener('input', () => this.request(suffix, Number(slider.value)));

    const nudge = (direction) => {
      const step = this.stepDeg / RAD2DEG;
      const here = Number(slider.value) / step;
      const next = direction > 0 ? Math.floor(here + 1e-6) + 1 : Math.ceil(here - 1e-6) - 1;
      this.request(suffix, next * step);
    };
    const minus = document.createElement('button');
    minus.className = 'jr-step';
    minus.textContent = '\u2212';
    minus.addEventListener('click', () => nudge(-1));
    const plus = document.createElement('button');
    plus.className = 'jr-step';
    plus.textContent = '+';
    plus.addEventListener('click', () => nudge(1));

    const value = document.createElement('input');
    value.className = 'jr-value';
    value.type = 'text';
    value.inputMode = 'decimal';
    value.title = '각도를 직접 입력';
    const commit = () => {
      const typed = Number(value.value.replace(/[^\d.+-]/g, ''));
      if (Number.isFinite(typed)) this.request(suffix, typed / RAD2DEG);
      else this.apply();

      if (value.dataset.shown !== undefined) value.value = value.dataset.shown;
    };
    value.addEventListener('change', commit);
    value.addEventListener('keydown', (event) => { if (event.key === 'Enter') value.blur(); });
    const unit = document.createElement('span');
    unit.className = 'jr-unit';
    unit.textContent = '\u00b0';

    const zero = document.createElement('button');
    zero.className = 'jr-zero';
    zero.title = '이 관절만 0으로';
    zero.innerHTML = ROTATE_ICON;
    zero.addEventListener('click', () => this.request(suffix, 0));

    row.append(name, minus, slider, plus, value, unit, zero);
    this.sliders.set(suffix, slider);
    this.valueLabels.set(suffix, value);
    return row;
  }

  buildPassiveRow(suffix) {
    const row = document.createElement('div');
    row.className = 'jogrow passive';

    const name = document.createElement('span');
    name.className = 'jr-label';
    name.textContent = suffix.replace(/^[a-z]+_/, '');

    const tag = document.createElement('span');
    tag.className = 'jr-passive';
    tag.textContent = 'passive';
    tag.title = 'driven by joint3 through the four-bar, not commanded';

    const value = document.createElement('span');
    value.className = 'jr-value';
    const unit = document.createElement('span');
    unit.className = 'jr-unit';
    unit.textContent = '\u00b0';

    row.append(name, tag, value, unit);
    this.valueLabels.set(suffix, value);
    return row;
  }

  request(suffix, radians) {
    const joint = this.jointOf(suffix);
    const room = this.reachLimit.get(suffix);
    const lower = Math.max(joint.limit.lower, room ? room.lower : -Infinity);
    const upper = Math.min(joint.limit.upper, room ? room.upper : Infinity);
    this.sliders.get(suffix).value = String(Math.max(lower, Math.min(upper, radians)));
    this.apply();
  }

  apply() {
    for (const group of GROUPS) {
      const isThumb = group.prefix === 'thumb';

      const abductionSuffix = `${group.prefix}_joint${isThumb ? 2 : 1}`;
      const flexionSuffix = `${group.prefix}_joint${isThumb ? 1 : 2}`;

      const flexionSlider = this.sliders.get(flexionSuffix);
      const abductionSlider = this.sliders.get(abductionSuffix);

      const pulled = projectIntoWorkspace(isThumb, Number(flexionSlider.value),
                                          Number(abductionSlider.value));
      const reached = new Map([[flexionSuffix, pulled.flexionRad],
                               [abductionSuffix, pulled.abductionRad]]);

      const distalSuffix3 = `${group.prefix}_joint3`;
      const requested3 = Number(this.sliders.get(distalSuffix3).value);
      const floor3 = isThumb ? 0 : longDistalFloorRad(pulled.abductionRad, pulled.flexionRad);
      reached.set(distalSuffix3, Math.max(requested3, floor3));

      const open = new Map([
        [abductionSuffix, { lower: -abductionRoomRad(isThumb, pulled.flexionRad),
                            upper: abductionRoomRad(isThumb, pulled.flexionRad) }],
        [flexionSuffix, flexionRangeRad(isThumb, pulled.abductionRad)],
        [distalSuffix3, { lower: floor3, upper: Infinity }],
      ]);

      this.reachLimit.set(abductionSuffix, open.get(abductionSuffix));
      this.reachLimit.set(flexionSuffix, open.get(flexionSuffix));

      for (const index of group.joints) {
        const suffix = `${group.prefix}_joint${index}`;
        const requested = Number(this.sliders.get(suffix).value);
        this.setJoint(suffix, reached.has(suffix) ? reached.get(suffix) : requested,
                      requested, open.get(suffix));
      }

      const q3 = reached.get(distalSuffix3);
      const distalSuffix = `${group.prefix}_joint4`;
      const distal = this.jointOf(distalSuffix);
      if (distal) {
        const value = passiveDistalRad(isThumb, q3);
        distal.setJointValue(value);
        this.showValue(distalSuffix, value);
      }
    }
  }

  showValue(suffix, radians) {
    const label = this.valueLabels.get(suffix);
    if (!label) return;
    const text = (radians * RAD2DEG).toFixed(1);
    if (label.tagName !== 'INPUT') { label.textContent = text; return; }

    label.dataset.shown = text;
    if (document.activeElement !== label) label.value = text;
  }

  setJoint(suffix, radians, requested = radians, open = undefined) {
    const joint = this.jointOf(suffix);
    if (!joint) return;
    joint.setJointValue(radians);
    this.showValue(suffix, radians);

    const slider = this.sliders.get(suffix);
    const span = joint.limit.upper - joint.limit.lower;
    const at = (value) => {
      const fraction = span === 0 ? 0 : (value - joint.limit.lower) / span;
      return Math.max(0, Math.min(1, fraction)) * 100;
    };

    const rest = at(0);
    const now = at(radians);
    const paint = { '--fill-from': Math.min(rest, now), '--fill-to': Math.max(rest, now),
                    '--open-from': at(open ? open.lower : joint.limit.lower),
                    '--open-to': at(open ? open.upper : joint.limit.upper) };
    for (const [name, value] of Object.entries(paint)) {
      slider.style.setProperty(name, `${value.toFixed(2)}%`);
    }

    const apart = Math.abs(radians - requested) * RAD2DEG > 0.05;
    slider.closest('.jogrow').classList.toggle('coupled', apart);
    const label = this.valueLabels.get(suffix);
    if (label) {
      label.title = apart
        ? `${(requested * RAD2DEG).toFixed(1)}° 요청, 기구가 허용하는 자세는 `
          + `${(radians * RAD2DEG).toFixed(1)}°`
        : '';
    }
  }
}

async function main() {
  const canvas = document.getElementById('view');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(
      getComputedStyle(document.documentElement).getPropertyValue('--viewer-bg').trim() || '#f0f0ee');

  const camera = new THREE.PerspectiveCamera(35, 1, 0.01, 10);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;

  const homePosition = new THREE.Vector3();
  const homeTarget = new THREE.Vector3();
  const goHome = () => {
    camera.position.copy(homePosition);
    controls.target.copy(homeTarget);
    controls.update();
  };
  document.getElementById('reset-view').addEventListener('click', goHome);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 0.9));
  const key = new THREE.DirectionalLight(0xffffff, 1.8);
  key.position.set(0.6, 0.8, 0.5);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xffffff, 0.35);
  fill.position.set(-0.7, 0.2, -0.6);
  scene.add(fill);

  const status = document.getElementById('status');
  let hands;
  try {
    hands = await Promise.all([loadHand('left'), loadHand('right')]);
  } catch (error) {
    status.textContent = `모델을 불러오지 못했습니다 — ${error.message}`;
    return;
  }
  status.remove();

  const frame = new THREE.Group();
  frame.rotation.x = -Math.PI / 2;
  scene.add(frame);

  const axes = new THREE.AxesHelper(AXES_LENGTH_M);
  axes.setColors(new THREE.Color(0xff0000), new THREE.Color(0x00c800), new THREE.Color(0x0040ff));
  frame.add(axes);

  const [left, right] = hands;
  left.position.y = -HAND_SEPARATION_M / 2;
  right.position.y = HAND_SEPARATION_M / 2;
  frame.add(left, right);

  new Hand('left', left, document.getElementById('left-panel'));
  new Hand('right', right, document.getElementById('right-panel'));

  frame.updateMatrixWorld(true);
  const reach = new THREE.Box3().setFromObject(frame);
  reach.getCenter(homeTarget);

  const corners = [];
  for (const x of [reach.min.x, reach.max.x]) {
    for (const y of [reach.min.y, reach.max.y]) {
      for (const z of [reach.min.z, reach.max.z]) corners.push(new THREE.Vector3(x, y, z));
    }
  }

  const placeHome = () => {
    const tanUp = Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2);
    const tanAcross = tanUp * camera.aspect;
    const across = new THREE.Vector3().crossVectors(ISO_VIEW, WORLD_UP).normalize();
    const up = new THREE.Vector3().crossVectors(across, ISO_VIEW).normalize();

    reach.getCenter(homeTarget);
    const rel = new THREE.Vector3();
    let distance = 0;

    for (let pass = 0; pass < 3; pass += 1) {

      distance = 0;
      for (const point of corners) {
        rel.copy(point).sub(homeTarget);
        const depth = rel.dot(ISO_VIEW);
        distance = Math.max(distance,
                            depth + Math.abs(rel.dot(across)) / tanAcross,
                            depth + Math.abs(rel.dot(up)) / tanUp);
      }
      distance *= FIT_MARGIN;

      let lowAcross = Infinity, highAcross = -Infinity;
      let lowUp = Infinity, highUp = -Infinity;
      for (const point of corners) {
        rel.copy(point).sub(homeTarget);
        const depth = distance - rel.dot(ISO_VIEW);
        const sideways = rel.dot(across) / depth;
        const upright = rel.dot(up) / depth;
        lowAcross = Math.min(lowAcross, sideways);
        highAcross = Math.max(highAcross, sideways);
        lowUp = Math.min(lowUp, upright);
        highUp = Math.max(highUp, upright);
      }
      homeTarget.addScaledVector(across, (lowAcross + highAcross) / 2 * distance)
                .addScaledVector(up, (lowUp + highUp) / 2 * distance);
    }
    homePosition.copy(ISO_VIEW).multiplyScalar(distance).add(homeTarget);
  };

  const resize = (recentre = false) => {
    const { clientWidth, clientHeight } = canvas.parentElement;
    renderer.setSize(clientWidth, clientHeight, false);
    camera.aspect = clientWidth / clientHeight;
    camera.updateProjectionMatrix();

    const settled = recentre || camera.position.distanceTo(homePosition) < 1e-6;
    placeHome();
    if (settled) goHome();
  };
  window.addEventListener('resize', () => resize());
  resize(true);

  renderer.setAnimationLoop(() => {
    controls.update();
    renderer.render(scene, camera);
  });
}

main();
