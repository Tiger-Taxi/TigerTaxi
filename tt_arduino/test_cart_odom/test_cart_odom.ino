#include "cart_odom.h"

ROSdue ros;

void setup() {
  ros.setup();
  setup_cart_odom(ros);
}

void loop() {
  cart_odom_spin();
}
