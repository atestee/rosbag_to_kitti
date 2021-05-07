import rospy
from tf2_py import ExtrapolationException, LookupException

# Subt artifacts: https://www.subtchallenge.com/resources/SubT_Cave_Artifacts_Specification.pdf
artifacts = ['backpack', 'phone', 'rescue_randy', 'rope', 'helmet', 'extinguisher', 'drill', 'vent']

def lookup_transforms_to_artifacts(msg, tf_buffer):
    transforms = []

    for artifact in artifacts:
        for i in range(1, 5):
            if tf_buffer.can_transform_core(artifact  + '_' + str(i), msg.header.frame_id, msg.header.stamp):
                try:
                    transforms.append(tf_buffer.lookup_transform_core(artifact + '_' + str(i), msg.header.frame_id, msg.header.stamp))
                except ExtrapolationException:
                    # print("Extrapolation exception between " + str(msg.header.frame_id) + " and " + artifact + '_' + str(i))
                    pass
                except LookupException:
                    # print("Lookup exception between " + str(msg.header.frame_id) + " and " + artifact + '_' + str(i))
                    pass

    return transforms

def save_transforms(transforms, name):
    file = open(name, 'w+')
    for tf in transforms:
        for line in str(tf).split('\n'):
            file.write(line + '\n')
        file.write('\n')
    file.close()

def save_artifact_data(name, tf_buffer):
    transforms = []

    for artifact in artifacts:
        for i in range(1, 5):
            if tf_buffer._frameExists(artifact + '_' + str(i)):
                transforms.append(tf_buffer.lookup_transform_core(artifact + '_' + str(i), 'world', rospy.Time(0)))

    file = open(name, 'w')
    for tf in transforms:
        for line in str(tf).split('\n'):
            file.write(line + '\n')
        file.write('\n')
    file.close()
