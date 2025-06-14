void init_compass() {
  qmc.init();
}

void update_compass() {
  qmc.read(&x1, &y1, &z1, &heading);
}

float true_heading() {
  float true_head = heading + 50;
  if (true_head > 360.0) true_head -= 360.0;
  return true_head;
}