# MaxAutoParticipantIndex: same budget as the C++ ChannelFactory. On lo (no multicast) every DDS process on
# the host takes a participant index, and Cyclone's default of 9 is used up by the robot stack with vision.
ChannelConfigHasInterface = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="$__IF_MULTICAST__$"/>
                </Interfaces>
            </General>
            <Discovery>
                <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
            </Discovery>
        </Domain>
    </CycloneDDS>'''

ChannelConfigAutoDetermine = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface autodetermine=\"true\" priority=\"default\" multicast=\"default\" />
                </Interfaces>
            </General>
            <Discovery>
                <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
            </Discovery>
        </Domain>
    </CycloneDDS>'''
