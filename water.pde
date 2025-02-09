class water {
    public
    int nWPts;
    PVector wPts[]; // Points making surface of the water.
    // vertical coordinates of wPts can move up and down like an elastic medium.
    float[] vp, fp, base; // vertical velocity, force and base height for each point on water surface.
    float yMean; // mean height of water

    water (int npts) {
        nWPts = npts;
        wPts = new PVector[nWPts];
        fp = new float[nWPts];
        vp = new float[nWPts];
        base = new float[nWPts];
        float tmpX, tmpY;
        for (int i = 0; i < nWPts; ++i) {
            // fp[i] = new PVector(0.0, 0.0);
            // vp[i] = new PVector(0.0, 0.0);
            fp[i] = 0.0; vp[i] = 0.0;
            tmpX = float(i)/(nWPts-1);
            yMean = 0.7*height;
            tmpY = yMean + 0.0*height*cos(6*PI*tmpX)*sin(2*PI*tmpX);//*cos(3.0*2*PI*tmpX);
            wPts[i] = new PVector(tmpX*width, tmpY);   
            base[i] = wPts[i].y;
        }
    }
    void setFwater(){
    // compute elastic forces between wPts to simulate waves on water.
        float lEq; //stiffness of water
        for (int i = 0; i < nWPts; ++i) {
            fp[i] = cWater*(yMean - wPts[i].y)/yMean;
            for (int j = i-1; j <= i+1; j+=2) {
                lEq = float(width)/(nWPts-1.0);
                if(j<0 || j>=nWPts) continue;
                fp[i] += cWater*(wPts[j].y - wPts[i].y)/lEq;
            }
            // add damping:
            fp[i] -= 0.0003*cWater*vp[i];
        }
                
    }

    void updatewater(){
    // update surface of the water.
        for (int i = 0; i < nWPts; ++i) {
            vp[i] += fp[i]*dt;
            wPts[i].y += vp[i]*dt;    
        }
    }

    void drawwater(){
    // draw water 
        fill(116,204,244);
        beginShape();
        vertex(0, height);
        for(PVector vec: wPts)  vertex(vec.x, vec.y);
        vertex(width, height);
        endShape();
    }
}