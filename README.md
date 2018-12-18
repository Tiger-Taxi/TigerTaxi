## Welcome to the Den of the TigerTaxi
In this project, we produced a framework for autonomy for the Tiger Taxi,
formerly known as the Autonomous People Mover. Whereas previous teams have
focused on local cost-based or ad-hoc navigation, we have attended to the lack
of global mapping and localization. Through a set of bag file-based and
stage-based simulation environments, as well as preliminary path planning, we
have produced a lucrative platform for point-to-point autonomous navigation. A
baseline navigation configuration for simple goals demonstrates the path
planning potential of the framework. We utilize the probablistic Octomap mapping
package to accumulate filtered laser scans. Global localization estimates supply
the module with the transform required to align the point clouds with previously
collected data. The ENet convolutional neural network is trained to classify
both unsafe and safe regions of the observable terrain. Where the LiDAR fails to
detect, e.g., subtle curbs, Safezone is able to compensate. The GUI facilitates
goal-setting and debugging of the Tiger Taxi subsystems. A global reference
frame indicates the cart's position with various windows for configuration,
monitoring, mode-setting, and horn noises.

[View documentation](http://edge.rit.edu/edge/C18501/public/Home) of the
TigerTaxi project.
