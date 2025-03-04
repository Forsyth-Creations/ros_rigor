
from hermes_robot.swerve_math import ModuleKinematics
from pytest import approx

def degToRad(deg):
    return deg * (3.14159 / 180)
    
zero_deg = degToRad(0)
fourty_five_deg = degToRad(45)
ninety_deg = degToRad(90)
one_thirty_five_deg = degToRad(135)
one_eighty_deg = degToRad(180)
two_seventy_deg = degToRad(270)
three_sixty_deg = degToRad(360)
four_fifty_deg = degToRad(450)

def test_shortest_angle():
    # Shortest angle should give you the most immediate angle to turn to
    assert ModuleKinematics.shortest_angle(zero_deg, ninety_deg) == approx(ninety_deg, rel=1e-5)
    assert ModuleKinematics.shortest_angle(zero_deg, fourty_five_deg) == approx(fourty_five_deg, rel=1e-5)
    assert ModuleKinematics.shortest_angle(zero_deg, two_seventy_deg) == approx(-ninety_deg, rel=1e-5)
    assert ModuleKinematics.shortest_angle(two_seventy_deg, zero_deg) == approx(three_sixty_deg, rel=1e-5)
    assert ModuleKinematics.shortest_angle(one_eighty_deg, zero_deg) == approx(zero_deg, rel=1e-5)
    
    
def test_normalize_angle():
    assert ModuleKinematics.normalize_angle(zero_deg) == approx(zero_deg, rel=1e-5)
    assert ModuleKinematics.normalize_angle(fourty_five_deg) == approx(fourty_five_deg, rel=1e-5)
    assert ModuleKinematics.normalize_angle(ninety_deg) == approx(ninety_deg, rel=1e-5)
    assert ModuleKinematics.normalize_angle(one_thirty_five_deg) == approx(one_thirty_five_deg, rel=1e-5)
    assert ModuleKinematics.normalize_angle(one_eighty_deg) == approx(one_eighty_deg, rel=1e-5)
    assert ModuleKinematics.normalize_angle(two_seventy_deg) == approx(two_seventy_deg, rel=1e-5)
    assert ModuleKinematics.normalize_angle(four_fifty_deg) == approx(ninety_deg, rel=1e-5)
    
    
def test_compute_wheel_angle():
    assert ModuleKinematics.compute_wheel_angle(0, 0, 1, 1, 1) == approx(fourty_five_deg + ninety_deg, rel=1e-5)
    assert ModuleKinematics.compute_wheel_angle(1, 0, 0, 1, 1) == approx(zero_deg, rel=1e-5)
    assert ModuleKinematics.compute_wheel_angle(0, 1, 0, 1, 1) == approx(ninety_deg, rel=1e-5)

def test_starting_degree():
    # Test 0 degree (as radians) angle
    x = 0.5
    y = 0.5
    z = 0.5
    theta = 0
    angularVelocity = 1 # 1 rad/s
    
    kinematics = ModuleKinematics("SwerveA", 20, {"x": x, "y": y, "z": z})
    assert kinematics.compute([0, 0, 0], angularVelocity, theta)[0] == approx(fourty_five_deg + ninety_deg, rel=1e-2)
    
    kinematics = ModuleKinematics("SwerveB", 20, {"x": x, "y": -y, "z": z})
    assert kinematics.compute([0, 0, 0], angularVelocity, theta)[0] == approx(fourty_five_deg, rel=1e-2)
    
    kinematics = ModuleKinematics("SwerveC", 20, {"x": -x, "y": -y, "z": z})
    assert kinematics.compute([0, 0, 0], angularVelocity, theta)[0] == approx(-fourty_five_deg, rel=1e-2)
    
    kinematics = ModuleKinematics("SwerveD", 20, {"x": -x, "y": y, "z": z})
    assert kinematics.compute([0, 0, 0], angularVelocity, theta)[0] == approx(-fourty_five_deg - ninety_deg, rel=1e-2)
    
    
def test_45_degree():
    
    # Test 0 degree (as radians) angle
    x = 0.5
    y = 0.5
    z = 0.5
    theta = 0
    angularVelocity = 1 # 1 rad/s
    
    kinematicsA = ModuleKinematics("SwerveA", 20, {"x": x, "y": y, "z": z})
    kinematicsB = ModuleKinematics("SwerveB", 20, {"x": x, "y": -y, "z": z})
    kinematicsC = ModuleKinematics("SwerveC", 20, {"x": -x, "y": -y, "z": z})
    kinematicsD = ModuleKinematics("SwerveD", 20, {"x": -x, "y": y, "z": z})
    
    # Note: "Compute" will capture the change, so we can then call it again later to get the new angle
    
    assert kinematicsA.compute([0, 0, 0], angularVelocity, theta)[0] == approx(fourty_five_deg + ninety_deg, rel=1e-2)
    assert kinematicsB.compute([0, 0, 0], angularVelocity, theta)[0] == approx(fourty_five_deg, rel=1e-2)
    assert kinematicsC.compute([0, 0, 0], angularVelocity, theta)[0] == approx(-fourty_five_deg, rel=1e-2)
    assert kinematicsD.compute([0, 0, 0], angularVelocity, theta)[0] == approx(-fourty_five_deg - ninety_deg, rel=1e-2)
    
    theta = fourty_five_deg
    
    assert kinematicsA.compute([0, 0, 0], angularVelocity, theta)[0] == approx(fourty_five_deg + ninety_deg, rel=1e-2)
    assert kinematicsB.compute([0, 0, 0], angularVelocity, theta)[0] == approx(fourty_five_deg, rel=1e-2)
    assert kinematicsC.compute([0, 0, 0], angularVelocity, theta)[0] == approx(-fourty_five_deg, rel=1e-2)
    assert kinematicsD.compute([0, 0, 0], angularVelocity, theta)[0] == approx(-fourty_five_deg - ninety_deg, rel=1e-2)
    

    
    