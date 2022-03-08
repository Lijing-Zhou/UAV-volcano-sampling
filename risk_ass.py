from std_msgs.msg import String

class RiskAssessment():

    def __init__(self):
        self.alt_border = 50

    # 3D border check
    def border_check(self, lat, lon, alt):
        """
        # index = 5
        # A, B, C ,D counter-clockwise
        self.flight_region = [[51.4234260, -2.6717208], [51.4212462, -2.6701340], 
                                [51.4224401, -2.6656878], [51.4246918, -2.6670602]]
        # A, B, C, D clockwise
        self.no_fly_zone = [[51.4224669, -2.6720597], [51.4235482, -2.6673723], 
                            [51.4228490, -2.6670013], [51.4217450, -2.6714016]]
        """

        # flight region border check
        lat_flight_ab = round((-137371 * lon * 10000000 + 47753256419832) / 1000000000000, 7)
        if lat < lat_flight_ab:
            return False
        lat_flight_bc = round((26852 * lon * 10000000 + 52138230581680) / 1000000000000, 7)
        if lat < lat_flight_bc:
            return False
        lat_flight_cd = round((-164070 * lon * 10000000 + 47048846126540) / 1000000000000, 7)
        if lat > lat_flight_cd:
            return False
        lat_flight_da = round((27160 * lon * 10000000 + 52149065369280) / 1000000000000, 7)
        if lat > lat_flight_da:
            return False
        
        # no fly zone border check
        lat_no_fly_ab = round((23068 * lon * 10000000 + 52038857631596) / 1000000000000, 7)
        lat_no_fly_bc = round((-188464 * lon * 10000000 + 46396511668528) / 1000000000000, 7)
        lat_no_fly_cd = round((25089 * lon * 10000000 + 52091972956157) / 1000000000000, 7)
        # lat_no_fly_da = round((-109695 * lon * 100000 + 48491351012085) / 1000000000000, 7)
        if lat < lat_no_fly_ab and lat < lat_no_fly_bc and lat > lat_no_fly_cd:
            return False

        if alt > 50:
            return False

        return True

    """
    safty check:
            arm_check
    """
    def arm_check(self):
        msg = String()
        msg.data = 'Please check if the UAV can be armed'
        return msg

    """
    sensor check:
            battery
    """
    # battery check 30%
    def battery_check(self, battery_msg):
        if battery_msg < 0.3:
            return False
        return True
    
    """
    complete_failure:
            GPS failure
            IMU failure
            Barometer failure
    """
    def complete_failure(self):
        # do nothing
        pass