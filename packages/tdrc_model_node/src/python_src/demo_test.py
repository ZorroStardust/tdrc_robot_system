from model import JointSpace, TDRCJointMotorModel

model = TDRCJointMotorModel(
    hole_radius=0.003,      # example
    spool_diameter=0.012,   # example
)

joint = JointSpace(
    phi_a=0.3,
    theta_a=0.8,
    phi_c=-0.4,
    theta_c=0.5,
)

res = model.joint_to_all(joint)

print("CC components:", res["cc"])
print("Tendon lengths:", res["tendon"])
print("Motor from tendon:", res["motor_from_tendon"])
print("Motor direct:", res["motor_direct"])

rec = model.motor_angles_to_joint(res["motor_direct"])
print("Recovered joint:", rec)
