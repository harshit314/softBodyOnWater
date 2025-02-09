void drawBdy(sftBdy bdy){
//draw body as polygon:
  fill(bdy.cVal);
  beginShape();
  for(PVector vec: bdy.coord){
    vertex(vec.x, vec.y);
  }
  endShape(CLOSE); 
  float tmp;
  for (int j = 1; j < nTrails; ++j) {
    stroke(70 + float((j+1)*100)/nTrails);
    // stroke(bdy.cVal, tmp);
    line(bdy.trails[j-1].x, bdy.trails[j-1].y, bdy.trails[j].x, bdy.trails[j].y);
  } 
}
