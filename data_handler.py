import rospy

def save_artifact_data(name, tf_buffer):
    transforms = []
    # Subt artifacts: https://www.subtchallenge.com/resources/SubT_Cave_Artifacts_Specification.pdf
    artifacts = ['backpack', 'phone', 'rescue_randy', 'rope', 'helmet', 'extinguisher', 'drill', 'vent']

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

def save_artifact_data2(name, tf_buffer, parent_frame, stamps):
    transforms = []
    s = len(stamps)

    # Subt artifacts: https://www.subtchallenge.com/resources/SubT_Cave_Artifacts_Specification.pdf
    artifacts = ['backpack', 'phone', 'rescue_randy', 'rope', 'helmet', 'extinguisher', 'drill', 'vent']

    for artifact in artifacts:
        for i in range(1, 5):
            if tf_buffer._frameExists(artifact + '_' + str(i)):
                for j in range(s):
                    t = rospy.Time(secs=stamps[j].secs, nsecs=stamps[j].nsecs)
                    transforms.append(tf_buffer.lookup_transform_core(artifact + '_' + str(i), parent_frame, t))

    file = open(name, 'w')
    for tf in transforms:
        for line in str(tf).split('\n'):
            file.write(line + '\n')
        file.write('\n')
    file.close()
