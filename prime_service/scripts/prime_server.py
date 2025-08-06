#!/usr/bin/env python

import rospy
from prime_service.srv import CountPrimes, CountPrimesResponse

def is_prime(n):
    if n <= 1:
        return False
    if n == 2:
        return True
    if n % 2 == 0:
        return False
    for i in range(3, int(n**0.5)+1, 2):
        if n % i == 0:
            return False
    return True

def handle_count_primes(req):
    a, b = req.a, req.b
    count = sum(1 for i in range(a, b + 1) if is_prime(i))
    rospy.loginfo(f"Client {req.client_id}: Counted {count} primes between {a} and {b}")
    return CountPrimesResponse(prime_count=count, client_id=req.client_id)

def prime_server():
    rospy.init_node('prime_server')
    service = rospy.Service('count_primes', CountPrimes, handle_count_primes)
    rospy.loginfo("Prime counting service ready.")
    rospy.spin()

if __name__ == "__main__":
    prime_server()
