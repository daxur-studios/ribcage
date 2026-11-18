/**
 * Purpose: the Three.js stage the 3D pages share: renderer, lights, orbit camera, the terrain mesh,
 * the instanced beams and nodes coloured by stress, and the skinned panels.
 * Value: `viewer3d.ts` (tower and bridge) and `craft3d.ts` (the craft page) draw the same way
 * from the core's render buffers, so a fix to drawing is made once. Drawing only reads.
 */

import {
  AmbientLight,
  BufferAttribute,
  BufferGeometry,
  Color,
  CylinderGeometry,
  DataTexture,
  DirectionalLight,
  DynamicDrawUsage,
  FloatType,
  HemisphereLight,
  InstancedMesh,
  Matrix4,
  Mesh,
  MeshStandardMaterial,
  MeshStandardNodeMaterial,
  NearestFilter,
  PerspectiveCamera,
  Quaternion,
  RGBAFormat,
  Scene,
  SphereGeometry,
  Vector3,
  WebGPURenderer,
} from 'three/webgpu';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { attribute, cross, Fn, int, ivec2, textureLoad, transformNormalToView, varyingProperty } from 'three/tsl';
import type { PanelSet } from '../src/panels3d.ts';
import { BEAM_ALIVE, BEAM_BROKEN, BEAM_YIELDED } from '../src/core.ts';
import { newPlacement, placeBeam, terrainTriangles } from '../src/render3d.ts';
import { NODE_TEXTURE_WIDTH, nodeTextureRows, packNodeTexture, skinAttributes } from '../src/skin.ts';
import { stressRgb, YIELDED_RGB } from '../src/stress-colour.ts';
import type { Terrain } from '../src/terrain.ts';
import type { Snapshot } from '../src/world.ts';

/** Radius of a drawn beam, metres. */
export const BEAM_DRAW_RADIUS = 0.07;

const HIDDEN = new Matrix4().makeScale(0, 0, 0);
/** What a beam is drawn as when the stress colours are switched off. */
const PLAIN_RGB: readonly [number, number, number] = [150, 156, 168];

export class Stage {
  readonly renderer: WebGPURenderer;
  readonly scene = new Scene();
  readonly camera = new PerspectiveCamera(45, 1, 0.1, 400);
  readonly controls: OrbitControls;
  readonly canvas: HTMLCanvasElement;

  terrainMesh: Mesh | null = null;
  beamMesh: InstancedMesh | null = null;
  nodeMesh: InstancedMesh | null = null;
  /** The skinned panels, and the texture their vertex function reads node positions from. */
  panelMesh: Mesh | null = null;
  nodeTexture: DataTexture | null = null;

  /** What is drawn; read when the meshes are made and when `applyLayers` is called. */
  showBeams = true;
  showNodes = false;
  showPanels = true;
  /** Beams coloured by stress (true) or all one grey. */
  stressColours = true;

  readonly #place = newPlacement();
  readonly #m = new Matrix4();
  readonly #p = new Vector3();
  readonly #q = new Quaternion();
  readonly #s = new Vector3();
  readonly #c = new Color();

  constructor(canvas: HTMLCanvasElement, forceWebGL: boolean) {
    this.canvas = canvas;
    // WebGPU where the browser has it, WebGL 2 otherwise.
    this.renderer = new WebGPURenderer({ canvas, antialias: true, forceWebGL });
    this.scene.background = new Color(0x0b0e13);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.scene.add(new AmbientLight(0xffffff, 0.25));
    this.scene.add(new HemisphereLight(0xcfe0ff, 0x2a2f3a, 1.4));
    const sun = new DirectionalLight(0xffffff, 2.4);
    sun.position.set(18, 30, 12);
    this.scene.add(sun);
  }

  /** Starts the renderer; returns which backend it got. */
  async init(): Promise<string> {
    await this.renderer.init();
    return (this.renderer.backend as { isWebGPUBackend?: boolean }).isWebGPUBackend ? 'WebGPU' : 'WebGL 2';
  }

  render(): void {
    void this.renderer.render(this.scene, this.camera);
  }

  resize(): void {
    const r = this.canvas.getBoundingClientRect();
    const w = Math.max(1, Math.round(r.width));
    const h = Math.max(1, Math.round(r.height));
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  /** Puts the camera at `from` looking at `to`. */
  setView(from: readonly [number, number, number], to: readonly [number, number, number]): void {
    this.camera.position.set(...from);
    this.controls.target.set(...to);
    this.controls.update();
  }

  #dispose(m: Mesh | null): void {
    if (!m) return;
    this.scene.remove(m);
    m.geometry.dispose();
  }

  /** The ground, from the same triangles the core uses. */
  showTerrain(t: Terrain): void {
    this.#dispose(this.terrainMesh);
    const { positions, indices } = terrainTriangles(t);
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(positions, 3));
    g.setIndex(new BufferAttribute(indices, 1));
    // Vertex colours: lower ground darker, so a valley reads as a valley.
    const colours = new Float32Array(t.nx * t.nz * 3);
    for (let i = 0; i < t.nx * t.nz; i++) {
      const h = positions[i * 3 + 1]!;
      const k = Math.max(0, Math.min(1, (h + 5) / 6));
      colours[i * 3] = 0.16 + 0.2 * k;
      colours[i * 3 + 1] = 0.2 + 0.22 * k;
      colours[i * 3 + 2] = 0.17 + 0.14 * k;
    }
    g.setAttribute('color', new BufferAttribute(colours, 3));
    g.computeVertexNormals();
    this.terrainMesh = new Mesh(g, new MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 1, metalness: 0 }));
    this.scene.add(this.terrainMesh);
  }

  /** One instanced cylinder per beam and one sphere per node, sized for this world. */
  makeInstances(nodes: number, beams: number, nodeRadius: number): void {
    this.#dispose(this.beamMesh);
    this.#dispose(this.nodeMesh);
    const cyl = new CylinderGeometry(BEAM_DRAW_RADIUS, BEAM_DRAW_RADIUS, 1, 8, 1);
    this.beamMesh = new InstancedMesh(cyl, new MeshStandardMaterial({ roughness: 0.55, metalness: 0.1 }), beams);
    this.beamMesh.instanceMatrix.setUsage(DynamicDrawUsage);
    this.beamMesh.frustumCulled = false;
    this.beamMesh.setColorAt(0, new Color(1, 1, 1)); // makes the colour buffer
    this.beamMesh.visible = this.showBeams;
    this.scene.add(this.beamMesh);
    this.nodeMesh = new InstancedMesh(new SphereGeometry(nodeRadius, 12, 8), new MeshStandardMaterial({ color: 0xcfd6e2, roughness: 0.6 }), nodes);
    this.nodeMesh.instanceMatrix.setUsage(DynamicDrawUsage);
    this.nodeMesh.frustumCulled = false;
    this.nodeMesh.visible = this.showNodes;
    this.scene.add(this.nodeMesh);
  }

  /** The TSL vertex function of `src/skin.ts` (`shadeVertexF32` is its CPU copy). */
  #skinMaterial(tex: DataTexture): MeshStandardNodeMaterial {
    // Read the vertex's four nodes from the node texture, place the vertex at `Σ wᵢ pᵢ`, and turn
    // its normal by the cofactor of the current edges `eᵢ = pᵢ − p₀`: `m₁ (e₂×e₃) + m₂ (e₃×e₁) + m₃ (e₁×e₂)`.
    const m = new MeshStandardNodeMaterial({ vertexColors: true, roughness: 0.85, metalness: 0 });
    const W = int(NODE_TEXTURE_WIDTH);
    const nodeAt = (i: ReturnType<typeof int>) => textureLoad(tex, ivec2(i.mod(W), i.div(W))).xyz;
    const skinNormal = varyingProperty('vec3', 'skinNormal');
    m.positionNode = Fn(() => {
      const idx = attribute('cellNodes', 'vec4');
      const w = attribute('cellWeights', 'vec4');
      const r = attribute('restNormal', 'vec3');
      const p0 = nodeAt(int(idx.x)).toVar();
      const p1 = nodeAt(int(idx.y)).toVar();
      const p2 = nodeAt(int(idx.z)).toVar();
      const p3 = nodeAt(int(idx.w)).toVar();
      const e1 = p1.sub(p0).toVar();
      const e2 = p2.sub(p0).toVar();
      const e3 = p3.sub(p0).toVar();
      skinNormal.assign(cross(e2, e3).mul(r.x).add(cross(e3, e1).mul(r.y)).add(cross(e1, e2).mul(r.z)));
      return p0.mul(w.x).add(p1.mul(w.y)).add(p2.mul(w.z)).add(p3.mul(w.w));
    })();
    m.normalNode = transformNormalToView(skinNormal).normalize();
    return m;
  }

  /** The panels as one mesh, bound to a node texture sized for `nodeCount` nodes. */
  makePanels(set: PanelSet, nodeCount: number): void {
    this.#dispose(this.panelMesh);
    this.nodeTexture?.dispose();
    const rows = nodeTextureRows(nodeCount);
    const tex = new DataTexture(new Float32Array(NODE_TEXTURE_WIDTH * rows * 4), NODE_TEXTURE_WIDTH, rows, RGBAFormat, FloatType);
    tex.minFilter = NearestFilter;
    tex.magFilter = NearestFilter;
    tex.generateMipmaps = false;
    this.nodeTexture = tex;
    const a = skinAttributes(set.binding);
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(Float32Array.from(set.positions), 3));
    g.setAttribute('normal', new BufferAttribute(Float32Array.from(set.normals), 3));
    g.setAttribute('color', new BufferAttribute(set.colours, 3));
    g.setAttribute('cellNodes', new BufferAttribute(a.nodes, 4));
    g.setAttribute('cellWeights', new BufferAttribute(a.weights, 4));
    g.setAttribute('restNormal', new BufferAttribute(a.restNormal, 3));
    g.setIndex(new BufferAttribute(set.indices, 1));
    this.panelMesh = new Mesh(g, this.#skinMaterial(tex));
    this.panelMesh.frustumCulled = false; // the rest-pose bounds say nothing about where the nodes are
    this.panelMesh.visible = this.showPanels;
    this.scene.add(this.panelMesh);
  }

  /** Shows or hides the three layers as `showBeams`, `showNodes` and `showPanels` say. */
  applyLayers(): void {
    if (this.beamMesh) this.beamMesh.visible = this.showBeams;
    if (this.nodeMesh) this.nodeMesh.visible = this.showNodes;
    if (this.panelMesh) this.panelMesh.visible = this.showPanels;
  }

  /** Uploads the drawn node positions for the panels' vertex function. */
  drawPanels(positions: Float32Array): void {
    if (!this.panelMesh || !this.nodeTexture || !this.panelMesh.visible) return;
    packNodeTexture(positions, this.nodeTexture.image.data as Float32Array);
    this.nodeTexture.needsUpdate = true;
  }

  /** Copies a snapshot's beams and flags, with node positions `positions` (blended between steps), into the instance buffers. */
  drawSnapshot(s: Snapshot, positions: Float32Array): void {
    const { beamMesh, nodeMesh } = this;
    if (!beamMesh || !nodeMesh) return;
    const { beams, nodeFlags } = s;
    const place = this.#place;
    const n = beams.flags.length;
    for (let i = 0; i < n; i++) {
      const f = beams.flags[i]!;
      if (!(f & BEAM_ALIVE) || f & BEAM_BROKEN) {
        beamMesh.setMatrixAt(i, HIDDEN);
        continue;
      }
      placeBeam(positions, beams.ends[i * 2]!, beams.ends[i * 2 + 1]!, place);
      this.#p.set(place.x, place.y, place.z);
      this.#q.set(place.qx, place.qy, place.qz, place.qw);
      this.#s.set(1, place.length, 1);
      beamMesh.setMatrixAt(i, this.#m.compose(this.#p, this.#q, this.#s));
      const [r, g, b] = !this.stressColours ? PLAIN_RGB : f & BEAM_YIELDED ? YIELDED_RGB : stressRgb(beams.ratio[i]!);
      beamMesh.setColorAt(i, this.#c.setRGB(r / 255, g / 255, b / 255));
    }
    beamMesh.instanceMatrix.needsUpdate = true;
    if (beamMesh.instanceColor) beamMesh.instanceColor.needsUpdate = true;
    if (nodeMesh.visible) {
      for (let i = 0; i < nodeFlags.length; i++) {
        if (!(nodeFlags[i]! & 1)) {
          nodeMesh.setMatrixAt(i, HIDDEN);
          continue;
        }
        this.#m.makeTranslation(positions[i * 3]!, positions[i * 3 + 1]!, positions[i * 3 + 2]!);
        nodeMesh.setMatrixAt(i, this.#m);
      }
      nodeMesh.instanceMatrix.needsUpdate = true;
    }
  }
}
