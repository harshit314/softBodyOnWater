class sftBdy{
  public
  int nPts; // number of vertices
  PVector coord[], fc[], vc[]; // coord is vertices coordinates, fc is force on each vertex, vc is velocity of each vertex
  PVector cntr; // centre of the body
  float lEq, rad; // equilibrium length and radius of body
  color cVal; // color of the body
  float theta; // angle for directed motion (from horizontal)
  PVector trails[];

  sftBdy(int nP, PVector xc, float radius, color cV){
  // constructor of soft body
    int len;
    float vinit;
    PVector tmp = new PVector(0, 0);
    nPts = nP>nPMin?nP:nPMin;
    coord = new PVector[nPts];
    fc = new PVector[nPts];
    vc = new PVector[nPts];
    len = coord.length;
    cntr = xc;
    // create circular body:
    rad = radius;
    cVal = cV;

    for(int i = 0; i<len; i++){
      tmp.set(rad*cos(i*2*PI/len), rad*sin(i*2*PI/len));
      coord[i] = PVector.add(xc,tmp);
      vc[i] = new PVector(0.0, 0.0);
      fc[i] = new PVector(0.0, 0.0);
    }
    lEq = 2*sin(PI/len)*(rad);
    theta = -PI/2.0 + random(-thMax, thMax);
    // to draw cool trails:
    trails = new PVector[nTrails];
    for (int i = 0; i < nTrails; ++i) {
      trails[i] = new PVector(0.0, 0.0);
      trails[i].set(cntr);
    }
  }
  
  void calcIntForces(){
  // compute internal forces for the body, like spring force and gravity:
    PVector dir = new PVector(0.0, 0.0), sc = new PVector(0.5*width, 0.5*height);
    PVector tmp2;
    float mag;
    int nM = this.coord.length, nhlf = min((nPMin-1)/2, (nM-1)/2);
    // apply spring force on each node which is connected to its neighbours:
    cntr = new PVector(0.0, 0.0);
    for(int i = 0; i<nM; i++){
      cntr = PVector.add(coord[i], cntr);
    }
    cntr.mult(1.0/nM); // gives central point
    theta += random(-thMax*sqDt, thMax*sqDt); // update motion direction
    for(int i = 0; i<nM; i++){
      fc[i].set(0.0, 0.0);
      fc[i].add(fActive*cos(theta), fActive*sin(theta));
      for(int j = i-nhlf; j<=i+nhlf; j++){
        if(j==i) continue;
        lEq = 2*sin(abs(j-i)*PI/nM)*(rad);
        dir = PVector.sub(this.coord[IX(j, nM)], this.coord[i]);
        mag = dir.mag();
        dir = dir.mult(1.0/mag);
        fc[i] = PVector.add(fc[i], PVector.mult(dir, ks*(mag-lEq)));
        // add damping force:
        tmp2 = PVector.sub(this.vc[IX(j, nM)], this.vc[i]);
        mag = tmp2.dot(dir);
        fc[i] = PVector.add(fc[i], PVector.mult(dir, 0.1*ks*mag));
      }    
      // add central point spring force:
      dir = PVector.sub(cntr, this.coord[i]);
      mag = dir.mag();
      dir = dir.mult(1.0/mag);
      fc[i].add(PVector.mult(dir, ks*(mag-rad)));
    }
    // add gravity:
    dir.set(0.0, gravity);
    // dir.set(PVector.sub(cntr, sc));
    for(int i = 0; i<nM; i++){      
      // fc[i].add(gravity*dir.y/width, -gravity*dir.x/width);
      fc[i].add(dir);
    }
  }
  
  void evolve(){
  // evolve position and velocity of each vertex using Newton's laws:
    int nM = coord.length;
    for(int i = 0; i<nM; i++){
      vc[i].add(PVector.mult(fc[i], dt));
      coord[i].add(PVector.mult(vc[i],dt));
    }
    for (int i = 1; i < nTrails; i++) {
      trails[i-1].set(trails[i]);
    }
    trails[nTrails-1].set(cntr);
  }
  
  void setFclick(float x, float y){
  // force upon clicking:
    PVector tmp = new PVector(x-cntr.x, y-cntr.y);
    float mgn;
    for(int i = 0; i<coord.length; i++){
      tmp.set(x - coord[i].x, y - coord[i].y);
      mgn = tmp.mag();
      if(mgn==0.0) continue;
      tmp.mult(fClick/mgn);
      fc[i].add(tmp);
    } 
  }

}

int IX(int i, int n){
// return the index i modulo n
  int res = i;
  if(i>=n)  res = i - n;
  else if(i<0) res = n + i;
  return res;
}

void calcExtForces(){
// calculate all the external forces on all the bodies. This includes inter-body forces, wall and water forces.
  PVector fR = new PVector(0.0, 0.0), dr = new PVector(0.0, 0.0);
  float mgn, rC, mgnC, fTmp;
  int n1, n2;
  // forces between different bodies:
  for(int i=0; i<nBody; i++){
    for(int j=i+1; j<nBody; j++){
      n1 = body[i].coord.length;
      n2 = body[j].coord.length;
      rC = (body[i].rad + body[j].rad);
      dr = PVector.sub(body[j].cntr, body[i].cntr);
      mgn = dr.mag();
      if(mgn>1.5*rC) continue; //bodies are far away...
      if(mgn<=rC){  // add centre to centre forces in case of large overlap:
        for(int k=0; k<body[i].coord.length; k++){
          mgnC = PVector.sub(body[j].cntr, body[i].coord[k]).mag();
          fTmp = exp(-0.1*(mgnC-mgn));
          fTmp = fTmp/(1.0+fTmp);
          body[i].fc[k].add(PVector.mult( dr, fTmp*kRep*(mgn-rC)/mgn));
        }
        for(int k=0; k<body[j].coord.length; k++){
          mgnC = PVector.sub(body[i].cntr, body[j].coord[k]).mag();
          fTmp = exp(-0.1*(mgnC-mgn));
          fTmp = fTmp/(1.0+fTmp);
          body[j].fc[k].sub(PVector.mult( dr, fTmp*kRep*(mgn-rC)/mgn));
        }
      }
    }
  }
  // forces due to uneven ground:
  PVector fvec = new PVector(0.0, 0.0);
  for(int i=0; i<nBody; i++){
    for(int j=0; j<wtr.nWPts; j++){
      n1 = body[i].coord.length;
      rC = body[i].rad;
      dr = PVector.sub(wtr.wPts[j], body[i].cntr);
      mgn = dr.mag();
      if(mgn>1.5*rC) continue; //bodies are far away...
      if(mgn<=rC){  // add centre to centre forces in case of large overlap:
        for(int k=0; k<body[i].coord.length; k++){
          mgnC = PVector.sub(wtr.wPts[j], body[i].coord[k]).mag();
          fTmp = exp(-0.1*(mgnC-mgn)); // the coefficient in the exponential controls variations of force across body's vertices 
          fTmp = fTmp/(1.0+fTmp);
          fvec = PVector.mult( dr, fTmp*kWall*(mgn-rC)/mgn);
          body[i].fc[k].add(fvec);
          wtr.fp[j] -= fWater*fvec.y; // its set before in updating ground function
        }      
      }
    }
  }
  
  // forces due to wall:
  mgn = wallOffst;
  for(int i=0; i<nBody; i++){
    for(int k=0; k<body[i].coord.length; k++){
      if(body[i].coord[k].x>width - mgn)  body[i].fc[k].add(kWall*(-body[i].coord[k].x+width-mgn), 0.0);
      if(body[i].coord[k].x<mgn)  body[i].fc[k].add(-kWall*(body[i].coord[k].x-mgn), 0.0);
      if(body[i].coord[k].y>height-mgn)  body[i].fc[k].add(0.0, kWall*(-body[i].coord[k].y+height-mgn));// more force from bottom wall.
      if(body[i].coord[k].y<mgn)  body[i].fc[k].add(0.0, -kWall*(body[i].coord[k].y-mgn)); 
    }
  }
}
