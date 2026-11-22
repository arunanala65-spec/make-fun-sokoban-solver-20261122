// Sokoban solver — push-optimal, room by room.
//
// Approach (own implementation): the level's rooms are independent — a box
// pushed through a door into the hallway can never come back, so every box
// stays in its room and the level's optimum is the sum of its rooms' optima.
// Each room is solved push-optimally with A* over pushes (state = the boxes
// plus the player's reachable region), with an admissible assignment
// heuristic, dead-square pruning and frozen 2x2-block pruning. The rooms'
// move strings are stitched together with the player's walks between them.
// Budgets are counts of states, never the clock, so runs are deterministic.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Directions: [dx, dy, walk letter]. Pushes reuse the same letter.
const DIRS = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];

// Work budgets (states), never the clock: runs stay deterministic.
const ROOM_STATES = 250000;
const GLOBAL_STATES = 2000000;
let expandedTotal = 0;

// ---------------------------------------------------------------- parsing
function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals = [];
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  // The outermost frame is wall whatever it shows: nothing leaves the board.
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// ------------------------------------------------------- reachability tools
// The lowest square reachable from `from`, avoiding walls and `occupied`
// boxes, flood-filling with `seen`/`stamp` so callers reuse the arrays.
function regionLow(w, wall, occupied, from, seen, stamp, stack) {
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    const a = p - w;
    if (!wall[a] && !occupied[a] && seen[a] !== stamp) { seen[a] = stamp; stack[n++] = a; }
    const b = p + w;
    if (!wall[b] && !occupied[b] && seen[b] !== stamp) { seen[b] = stamp; stack[n++] = b; }
    const c = p - 1;
    if (!wall[c] && !occupied[c] && seen[c] !== stamp) { seen[c] = stamp; stack[n++] = c; }
    const d = p + 1;
    if (!wall[d] && !occupied[d] && seen[d] !== stamp) { seen[d] = stamp; stack[n++] = d; }
  }
  return low;
}

// The squares from which a box alone can be pushed onto some goal: pull
// boxes back from the goals over open floor (a pull needs the square behind
// the player to be open too).
function liveSquares(w, wall, goals) {
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi];
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

// Walking distances from `from` to every square, ignoring boxes (BFS).
function bfsDist(w, wall, from) {
  const dist = new Int32Array(wall.length).fill(-1);
  const queue = [from];
  dist[from] = 0;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    for (const [dx, dy] of DIRS) {
      const q = p + dy * w + dx;
      if (!wall[q] && dist[q] < 0) {
        dist[q] = dist[p] + 1;
        queue.push(q);
      }
    }
  }
  return dist;
}

// The shortest walk from `from` to `to` around walls and boxes, in LURD
// letters (first direction first on ties, so the walk is deterministic).
function walkTo(w, wall, occ, from, to) {
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occ[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: push square out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

// The square the player stands on after `moves` from `from` on a board `w` wide.
function endOf(w, from, moves) {
  let p = from;
  for (const ch of moves) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

// ------------------------------------------------------------ deadlock cuts
// Whether the box just pushed to `t` sits in a frozen 2x2 block of walls
// and boxes with a box off its goal: no push can ever free such a block.
function frozenBlock(w, wall, occupied, goal, t) {
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b])) continue;
    if (!(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) ||
        (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

// ------------------------------------------------------------ the push search
// A* over pushes for one (sub-)level: a state is the boxes plus the player's
// reachable region (named by its lowest square). The heuristic is the cheapest
// assignment of boxes to goals by walking distance, which one push can lower
// by at most one, so weight 1 is push-optimal. Returns {pushes, moves} or
// null when the state budget runs out or the level has no solution.
function solveRoom(level, maxStates, weight) {
  const { w, wall, goals } = level;
  const size = wall.length;
  const n = level.boxes.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(w, wall, goals);
  if (level.boxes.some((b) => !live[b])) return null;
  // Walking distance from each goal to every square (boxes ignored).
  const dists = goals.map((g) => bfsDist(w, wall, g));
  const solved = (boxes) => boxes.every((b) => goal[b]);
  // Cheapest box-to-goal assignment: exact by permutations while small,
  // else the sum of each box's nearest goal (still admissible).
  function heuristic(boxes) {
    if (n === 0) return 0;
    if (n <= 6) {
      const used = new Uint8Array(n);
      let best = Infinity;
      const rec = (i, sum) => {
        if (sum >= best) return;
        if (i === n) { best = sum; return; }
        for (let g = 0; g < n; g++) {
          if (used[g]) continue;
          const d = dists[g][boxes[i]];
          if (d < 0) continue;
          used[g] = 1;
          rec(i + 1, sum + d);
          used[g] = 0;
        }
      };
      rec(0, 0);
      return best === Infinity ? 1e9 : best;
    }
    let sum = 0;
    for (const b of boxes) {
      let m = Infinity;
      for (let g = 0; g < n; g++) {
        const d = dists[g][b];
        if (d >= 0 && d < m) m = d;
      }
      if (m === Infinity) return 1e9;
      sum += m;
    }
    return sum;
  }

  const keyOf = (boxes, low) => {
    let k = low;
    for (const b of boxes) k = k * size + b;
    return k;
  };

  const occupied = new Uint8Array(size);
  // Two flood-fill mark arrays: `seen` holds the region of the state being
  // expanded (its pushes are read off it), `mark` the region of a successor
  // (its key) — they must not share stamps, or the key's flood would wipe
  // the region the push checks read.
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  let mstamp = 0;

  const sBoxes = [level.boxes.slice()];
  const sPlayer = [level.player];
  const sParent = [-1];
  const sDir = [-1];
  const sTo = [-1];
  const sG = [0];
  const sKey = [];
  const bestG = new Map();
  sKey.push(keyOf(level.boxes, regionLow(w, wall, occupied, level.player, mark, ++mstamp, stack)));
  bestG.set(sKey[0], 0);

  // Binary heap on [f, g, seq, state]: lowest f first, deeper first on ties,
  // then insertion order, so the search is fully deterministic.
  const heap = [];
  let seq = 0;
  const hLess = (a, b) => a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const hPush = (e) => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!hLess(heap[i], heap[p])) break;
      [heap[i], heap[p]] = [heap[p], heap[i]];
      i = p;
    }
  };
  const hPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && hLess(heap[c + 1], heap[c])) c++;
        if (!hLess(heap[c], heap[i])) break;
        [heap[i], heap[c]] = [heap[c], heap[i]];
        i = c;
      }
    }
    return top;
  };
  hPush([weight * heuristic(level.boxes), 0, seq++, 0]);

  let found = -1;
  while (heap.length > 0) {
    const [, g, , si] = hPop();
    if (g !== bestG.get(sKey[si])) continue; // a better route to it was found
    const boxes = sBoxes[si];
    if (solved(boxes)) { found = si; break; }
    if (expandedTotal >= GLOBAL_STATES) return null;
    expandedTotal++;
    for (const b of boxes) occupied[b] = 1;
    regionLow(w, wall, occupied, sPlayer[si], seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      for (let k = 0; k < 4; k++) {
        const [dx, dy] = DIRS[k];
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        if (frozenBlock(w, wall, occupied, goal, t)) {
          occupied[t] = 0;
          occupied[b] = 1;
          continue;
        }
        const next = boxes.slice();
        next[bi] = t;
        next.sort((a, x) => a - x);
        const key = keyOf(next, regionLow(w, wall, occupied, b, mark, ++mstamp, stack));
        occupied[t] = 0;
        occupied[b] = 1;
        const ng = g + 1;
        const known = bestG.get(key);
        if (known !== undefined && known <= ng) continue;
        if (sBoxes.length >= maxStates) {
          for (const x of boxes) occupied[x] = 0;
          return null;
        }
        bestG.set(key, ng);
        sBoxes.push(next);
        sPlayer.push(b);
        sParent.push(si);
        sDir.push(k);
        sTo.push(t);
        sG.push(ng);
        sKey.push(key);
        hPush([ng + weight * heuristic(next), ng, seq++, sBoxes.length - 1]);
      }
    }
    for (const x of boxes) occupied[x] = 0;
  }
  if (found < 0) return null;

  // The path of states, then the moves: walk to each push's square, then push.
  const path = [];
  for (let s = found; s > 0; s = sParent[s]) path.push(s);
  path.reverse();
  let moves = '';
  let player = level.player;
  let boxes = level.boxes;
  for (const s of path) {
    const nextBoxes = sBoxes[s];
    const from = boxes.find((b) => !nextBoxes.includes(b));
    const to = nextBoxes.find((b) => !boxes.includes(b));
    const d = to - from;
    const dir = DIRS.findIndex(([dx, dy]) => dy * w + dx === d);
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(w, wall, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += DIRS[dir][2];
    player = from;
    boxes = nextBoxes;
  }
  return { pushes: path.length, moves };
}

// --------------------------------------------------------- room decomposition
// The level's rooms: cut the dead squares of the player's row (the hallway:
// a box there reaches no goal) out of the floor, and each part left holding
// a box is a room. Null when the level is no set of rooms.
function roomsOf(level) {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(w, wall, goals);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i] + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [], done: false });
  }
  for (const b of boxes) rooms[part[b]].boxes.push(b);
  for (const g of goals) if (part[g] >= 0) rooms[part[g]].goals.push(g);
  if (goals.some((g) => part[g] < 0)) return null;
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

// ------------------------------------------------------------- level solving
// Solve room by room: each room's push-optimal solution in turn, the other
// rooms' boxes standing as walls (on their goals once their room is done),
// the player walking on from where the last room left it. A box never leaves
// its room, so the rooms' optima add up to the level's.
function solveLevel(level) {
  const rooms = roomsOf(level);
  if (rooms === null) {
    const whole = solveRoom(level, ROOM_STATES, 1) ?? solveRoom(level, ROOM_STATES, 2);
    return whole ? whole.moves : '';
  }
  let player = level.player;
  let moves = '';
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const other of rooms) {
      if (other === room) continue;
      for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
    }
    const sub = {
      w: level.w, h: level.h, wall,
      goals: room.goals.slice().sort((a, b) => a - b),
      boxes: room.boxes.slice().sort((a, b) => a - b),
      player,
    };
    const solved = solveRoom(sub, ROOM_STATES, 1) ?? solveRoom(sub, ROOM_STATES, 2);
    if (solved === null) return '';
    moves += solved.moves;
    player = endOf(level.w, player, solved.moves);
    room.done = true;
  }
  return moves;
}

// --------------------------------------------------------------------- driver
for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(parseLevel(readFileSync(join(I, id, 'level.txt'), 'utf8')));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
