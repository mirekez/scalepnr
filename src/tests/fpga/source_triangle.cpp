// Source Moving is exercised separately from Destination Moving. The shared
// fixture keeps the three-cell geometry/real owners, but reverses the work:
// the right-hand cell's output must reach the stationary register. Moving that
// source away from its co-located input driver must turn a LOCAL->LOCAL
// connection into fabric. It must reconnect that input
// input through fabric, as well as preserve the register's feedback input.
// Each family has a closed control (atomic failure) and a real fabric witness.
void run_moving_triangle(int family,bool open,bool source_moving);

void source_triangle_regression(int family,bool open) {
  run_moving_triangle(family,open,true);
}
