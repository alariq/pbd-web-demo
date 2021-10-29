"use strict";



var canvas = null;//document.getElementById("myCanvas");
var c = null;//canvas.getContext("2d");

var simMinWidth = 2.0;
var cScale = 1;//Math.min(canvas.width, canvas.height) / simMinWidth;
var simWidth = 1;//canvas.width / cScale;
var simHeight = 1;//canvas.height / cScale;

var doc = null;

function setupGlobals(canvas_, win, doc_) {
    canvas = canvas_;
    c = canvas.getContext("2d");
    canvas.width = win.innerWidth - 20;
    canvas.height = win.innerHeight - 100;

    simMinWidth = 2.0;
    cScale = Math.min(canvas.width, canvas.height) / simMinWidth;
    simWidth = canvas.width / cScale;
    simHeight = canvas.height / cScale;

    doc = doc_;
}

function cX(posX) {
    return posX * cScale;
}

function cY(posY) {
    return canvas.height - posY * cScale;
}

// vector math -------------------------------------------------------

function length(x0, y0, x1, y1) {
    return Math.sqrt((x0 - x1)*(x0-x1) + (y0-y1)*(y0-y1));
}

class Vector2 {
    constructor(x = 0.0, y = 0.0) {
        this.x = x; 
        this.y = y;
    }

    set(v) {
        this.x = v.x; this.y = v.y;
    }

    clone() {
        return new Vector2(this.x, this.y);
    }

    add(v, s = 1.0) {
        this.x += v.x * s;
        this.y += v.y * s;
        return this;
    }

    addVectors(a, b) {
        this.x = a.x + b.x;
        this.y = a.y + b.y;
        return this;
    }

    subtract(v, s = 1.0) {
        this.x -= v.x * s;
        this.y -= v.y * s;
        return this;
    }

    subtractVectors(a, b) {
        this.x = a.x - b.x;
        this.y = a.y - b.y;
        return this;			
    }

    length() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    scale(s) {
        this.x *= s;
        this.y *= s;
        return this;
    }

    dot(v) {
        return this.x * v.x + this.y * v.y;
    }

    perp() {
        return new Vector2(-this.y, this.x);
    }
}

class ConstraintsArray {
    static IS = 2;
    static DS = 7;

    constructor(size) { 
        this.Idx = new Int32Array(ConstraintsArray.IS*size); 
        this.Data1 = new Float32Array(ConstraintsArray.DS*size); 
        this.maxSize = size;
        this.size = 0;
    }

    clear() {
        this.size = 0;
    }

    pushBack2(A, B, d, dvx, dvy, nx, ny, mu_k, e) {
        if (this.size >= this.maxSize) {
            this.maxSize *= 2;
            var oldIdx = this.Idx;
            var oldData = this.Data1;

            this.Idx = new Int32Array(ConstraintsArray.IS*this.maxSize); 
            this.Data1 = new Float32Array(ConstraintsArray.ID*this.maxSize); 

            for (var i = 0; i < oldIdx.length; ++i)
            {
                this.Idx[i] = oldIdx[i];
            }
            for (i = 0; i < oldData.length; ++i)
            {
                this.Data1[i] = oldData[i];
            }
        }
        var is = ConstraintsArray.IS;
        var ds = ConstraintsArray.DS;
        this.Idx[is*this.size + 0] = A; 
        this.Idx[is*this.size + 1] = B; 

        this.Data1[ds*this.size + 0] = d; 
        this.Data1[ds*this.size + 1] = dvx; 
        this.Data1[ds*this.size + 2] = dvy; 
        this.Data1[ds*this.size + 3] = nx; 
        this.Data1[ds*this.size + 4] = ny; 
        this.Data1[ds*this.size + 5] = mu_k; 
        this.Data1[ds*this.size + 6] = e; 
        this.size++;
    }

    idxA(i) {
        return this.Idx[ConstraintsArray.IS*i+ 0]; 
    }
    idxB(i) {
        return this.Idx[ConstraintsArray.IS*i + 1]; 
    }
    d(i) {
        return this.Data1[ConstraintsArray.DS*i + 0]; 
    }
    dvx(i) {
        return this.Data1[ConstraintsArray.DS*i + 1]; 
    }
    dvy(i) {
        return this.Data1[ConstraintsArray.DS*i + 2]; 
    }
    nx(i) {
        return this.Data1[ConstraintsArray.DS*i + 3]; 
    }
    ny(i) {
        return this.Data1[ConstraintsArray.DS*i + 4]; 
    }
    mu_k(i) {
        return this.Data1[ConstraintsArray.DS*i + 5]; 
    }
    e(i) {
        return this.Data1[ConstraintsArray.DS*i + 6]; 
    }
}


class Vector {
    constructor(size, creator_fp) { 
        this.vals = new Int32Array(size); 
        this.maxSize = size;
        this.size = 0;
    }
    clear() {
        this.size = 0;
    }
    pushBack(val) {
        if (this.size >= this.maxSize) {
            this.maxSize *= 2;
            var old = this.vals;
            this.vals = new Int32Array(this.maxSize);
            for (var i = 0; i < old.length; i++)
                this.vals[i] = old[i];
        }
        this.vals[this.size++] = val;
    }
}

// scene -------------------------------------------------------

var physicsScene = 
    {
        gravity : new Vector2(0.0, -9.81),
        dt : 1.0 / 60.0,
        numSteps : 20,
        paused : false,
        numRows:  1,       
        numColumns:  1,
        numParticles: 0,
        boundaryCenter: new Vector2(0.0, 0.0),
        boundaryRadius: 0.0
    };

var particleRadius = 0.01;
var maxVel = 0.4 * particleRadius;
var mu_s = 0.2; // static friction
var mu_k = 0.2; // dynamic friction
var e = 0.2; // restitution

var maxParticles = 1000;

var particles = {
    pos : new Float32Array(2 * maxParticles),
    prev : new Float32Array(2 * maxParticles),
    vel : new Float32Array(2 * maxParticles)
}

var constraints = new ConstraintsArray(1);

// -------------------------------------------------------

function setupScene() 
{
    // init particle positions
    var particleDiameter = 2*particleRadius;

    if(physicsScene.numColumns*physicsScene.numRows > maxParticles)
        return;

    physicsScene.radius = simMinWidth * 0.05;
    physicsScene.paused = true;
    physicsScene.boundaryCenter.x = simWidth / 2.0;
    physicsScene.boundaryCenter.y = simHeight / 2.0;
    physicsScene.boundaryRadius = simMinWidth * 0.4;

    var offsetX = physicsScene.boundaryCenter.x - physicsScene.numColumns*particleDiameter*0.5;
    var offsetY = physicsScene.boundaryCenter.y + physicsScene.numRows*particleDiameter*0.5;

    offsetX += particleRadius * 50;

    var idx = 0;
    for (var y = 0; y < physicsScene.numRows; y++) {
        for (var x = 0; x < physicsScene.numColumns; x++) {
            // x
            particles.pos[idx] = offsetX + x * 1.2*particleDiameter;
            particles.pos[idx] += 0.01 * particleDiameter * (y % 2);
            // y
            particles.pos[idx+ 1] = offsetY - y * particleDiameter;

            // v.x v.y
            particles.vel[idx] = 0.0;
            particles.vel[idx + 1] = 0.0;
            idx += 2;				
        }
    }
    physicsScene.numParticles = physicsScene.numColumns * physicsScene.numRows;

    initNeighborsHash();
    constraints.clear();

    doc.getElementById("force").innerHTML = 0.0.toFixed(3);		
    doc.getElementById("aforce").innerHTML = 0.0.toFixed(3);		
}

// draw -------------------------------------------------------

function drawCircle(posX, posY, radius, filled)
{
    c.beginPath();			
    c.arc(
        cX(posX), cY(posY), cScale * radius, 0.0, 2.0 * Math.PI); 
    c.closePath();
    if (filled)
        c.fill();
    else 
        c.stroke();
}

function drawCircleV(pos, radius, filled)
{
    drawCircle(pos.x, pos.y, radius, filled);
}

function draw() 
{
    c.clearRect(0, 0, canvas.width, canvas.height);

    c.fillStyle = "#FF0000";
    c.lineWidth = 2.0;

    for(var i=0;i<physicsScene.numParticles; ++i)
    {
        var x = particles.pos[2*i + 0];
        var y = particles.pos[2*i + 1];
        drawCircle(x, y, particleRadius, true);
    }

    c.fillStyle = "#FF0000";

    drawCircleV(physicsScene.boundaryCenter, physicsScene.boundaryRadius, false);

    c.fillStyle = "#00FF00";
}

// ------------------------------------------------
function solveBoundaryConstraint()
{
    var br = physicsScene.boundaryRadius - particleRadius;
    var bc = physicsScene.boundaryCenter;

    var dir = new Vector2(0,0);
    var pi = new Vector2(0,0);
    var dp = new Vector2(0,0);
    var dp_n = new Vector2(0,0);
    var dp_t = new Vector2(0,0);
    var path = new Vector2(0,0);
    for(var i=0;i<physicsScene.numParticles; ++i)
    {
        pi.x = particles.pos[2*i + 0];
        pi.y = particles.pos[2*i + 1];

        dir.subtractVectors(bc, pi);
        var d = dir.length();
        dir.scale(1/d);
        var C = br - d;
        if(C < 0)
        {
            dp.x = 0;
            dp.y = 0;
            dp.add(dir, -C);

            // static friction

            // corrected pos minus old pos
            path.x = (particles.pos[2 * i + 0] + dp.x - particles.prev[2 * i + 0]);
            path.y = (particles.pos[2 * i + 1] + dp.y - particles.prev[2 * i + 1]);

            // perp projection
            var proj = path.dot(dir);
            dp_n.add(dir, proj);

            dp_t.subtractVectors(path, dp_n);
            var dp_t_len = dp_t.length();
            // part of velocity update pass
            if (dp_t_len  < mu_s * path.length()) {
                particles.pos[2*i + 0] -= dp_t.x;
                particles.pos[2*i + 1] -= dp_t.y;
            }

            doc.getElementById("dp_t_len").innerHTML = dp_t_len.toFixed(8);		

            particles.pos[2*i + 0] += dp.x;
            particles.pos[2*i + 1] += dp.y;


        }
    }
}

function solveCollisionConstraints()
{
    var dir = new Vector2(0,0);
    for(var i=0;i<physicsScene.numParticles; ++i)
    {
        var px = particles.pos[2 * i];
        var py = particles.pos[2 * i + 1];

        var first = firstNeighbor[i];
        var num = firstNeighbor[i + 1] - first;

        for (var j = 0; j < num; j++) {


            var id = neighbors.vals[first + j];				
            if(id == i)
                continue;

            var nx = particles.pos[2 * id] - px;				
            var ny = particles.pos[2 * id + 1] - py;
            var d = Math.sqrt(nx * nx + ny * ny);

            if (d > 0) {
                nx /= d;
                ny /= d;
            }

            var C = 2*particleRadius - d;
            if(C > 0) {
                // particles have same mass, so  w1/(w1+w2) = w2/(w1+w2) = 0.5

                var dx = -0.5*C*nx;
                var dy = -0.5*C*ny;

                particles.pos[2 * i] += dx;
                particles.pos[2 * i + 1 ]+= dy;

                particles.pos[2 * id] -= dx;				
                particles.pos[2 * id + 1] -= dy;

                //var temp = new Float32Array(particles.pos.length);
                //for(n=0; n<temp.length;++n) {
                //temp[n] = particles.pos[n];
                //}

                // fill constraint for velocity pass
                var dvx =0;// particles.vel[2 * i] - particles.vel[2 * id];
                var dvy =0;// particles.vel[2 * i + 1] - particles.vel[2 * id + 1];
                constraints.pushBack2();//i, id, C, dvx, dvy, nx, ny, mu_k, e);

                //for(n=0; n<temp.length;++n) {
                //if(temp[n]!=particles.pos[n]) {
                //var fffff=0;
                //	}
                //}

            }

        }
    }

}

// -----------------------------------------------------------------------------------

var hashSize = 370111;

var hash = {
    size : hashSize,

    first : new Int32Array(hashSize),
    marks : new Int32Array(hashSize),
    currentMark : 0,

    next : new Int32Array(maxParticles),

    orig : { left : -100.0, bottom : -1.0 }		
}

var gridSpacing = 2*5*particleRadius;
var invGridSpacing = 1/gridSpacing;
var firstNeighbor = new Int32Array(maxParticles + 1);
var neighbors = new Vector(10);// * maxParticles);

function initNeighborsHash() {
    for (var i = 0; i < hashSize; i++) {
        hash.first[i] = -1;
        hash.marks[i] = 0;
    }			
}

function findNeighbors() 
{
    // hash particles
    hash.currentMark++;

    for (var i = 0; i < physicsScene.numParticles; i++) {
        var px = particles.pos[2 * i];
        var py = particles.pos[2 * i + 1];

        var gx = Math.floor((px - hash.orig.left) * invGridSpacing);
        var gy = Math.floor((py - hash.orig.bottom) * invGridSpacing);

        var h = (Math.abs((gx * 92837111) ^ (gy * 689287499))) % hash.size;

        if (hash.marks[h] != hash.currentMark) {				
            hash.marks[h] = hash.currentMark;
            hash.first[h] = -1;
        }

        hash.next[i] = hash.first[h];
        hash.first[h] = i;
    }

    // collect neighbors
    neighbors.clear();

    var h2 = gridSpacing * gridSpacing;

    for (var i = 0; i < physicsScene.numParticles; i++) {
        firstNeighbor[i] = neighbors.size;

        var px = particles.pos[2 * i];
        var py = particles.pos[2 * i + 1];

        var gx = Math.floor((px - hash.orig.left) * invGridSpacing);
        var gy = Math.floor((py - hash.orig.bottom) * invGridSpacing);

        var x, y;

        for (x = gx - 1; x <= gx + 1; x++) {
            for (y = gy - 1; y <= gy + 1; y++) {

                var h = (Math.abs((x * 92837111) ^ (y * 689287499))) % hash.size;

                if (hash.marks[h] != hash.currentMark) 
                    continue;

                var id = hash.first[h];
                while (id >= 0) 
                {
                    var dx = particles.pos[2 * id] - px;
                    var dy = particles.pos[2 * id + 1] - py;

                    if (dx * dx + dy * dy < h2) 
                        neighbors.pushBack(id);

                    id = hash.next[id];						
                }
            }
        }
    }
    firstNeighbor[physicsScene.numParticles] = neighbors.size;
}

// ------------------------------------------------

function velocityUpdate()
{

}


// ------------------------------------------------

function simulate() 
{
    if (physicsScene.paused)
        return;

    var h = physicsScene.dt / physicsScene.numSteps;
    var force, analyticForce;
    var g = physicsScene.gravity;
    var force = 0;
    var vi = new Vector2(0,0);
    var pi = new Vector2(0,0);
    var pprevi = new Vector2(0,0);

    constraints.clear();
    findNeighbors();

    for (var step = 0; step < physicsScene.numSteps; step++)
    {
        constraints.clear();
        // predict
        for (var i = 0; i < physicsScene.numParticles; i++)
        {
            // use temp var to not overwrite correct velocities
            // we need them when filling constraint structure
            var vx = particles.vel[2 * i + 0] + g.x * h;
            var vy = particles.vel[2 * i + 1] + g.y * h;
            particles.prev[2 * i] = particles.pos[2 * i];
            particles.prev[2 * i + 1] = particles.pos[2 * i + 1];
            particles.pos[2 * i] += vx * h;
            particles.pos[2 * i + 1] += vy * h;
        }

        solveCollisionConstraints();
        solveBoundaryConstraint();

        //force = Math.abs(lambda / sdt / sdt);

        // update velocities
        for (i = 0; i < physicsScene.numParticles; i++)
        {
            pi.x = particles.pos[2*i + 0];
            pi.y = particles.pos[2*i + 1];

            pprevi.x = particles.prev[2*i + 0];
            pprevi.y = particles.prev[2*i + 1];

            vi.subtractVectors(pi, pprevi);
            vi.scale(1.0 / h);

            particles.vel[2 * i + 0] = vi.x;
            particles.vel[2 * i + 1] = vi.y;
        }

        velocityUpdate();

        //analyticForce = physicsScene.analyticBead.simulate(sdt, -physicsScene.gravity.y);
    }

    document.getElementById("force").innerHTML = force.toFixed(3);		
}

// --------------------------------------------------------

function run() {
    physicsScene.paused = false;
}

function step() {
    physicsScene.paused = false;
    simulate();
    physicsScene.paused = true;
}

function update() {
    simulate();
    draw();
    requestAnimationFrame(update);
}

//setupScene();
//update();
