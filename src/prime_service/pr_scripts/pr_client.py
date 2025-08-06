#!/usr/bin/env python

import rospy
from prime_service.srv import CountPrimes

def prime_client(a, b, client_id):
    rospy.wait_for_service("count_primes")
    try:
        count_primes = rospy.ServiceProxy("count_primes", CountPrimes)
        response = count_primes(a, b, client_id)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service error: %s" % e)

if __name__ == '__main__':
    rospy.init_node("prime_client")

    a = int(input("Enter starting number: "))
    b = int(input("Enter ending number: "))
    client_id = int(input("Enter cliend id: "))

    result = prime_client(a, b, client_id)
    rospy.loginfo("ClientId: %d | Prime Count: %d", result.client_id, result.prime_count)
