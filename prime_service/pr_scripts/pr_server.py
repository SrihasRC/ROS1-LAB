#!/usr/bin/env python

import rospy
from prime_service.srv import CountPrimes, CountPrimesResponse

def isPrime(n):
    if n<=1:
        return False
    if n==2:
        return True
    for i in range (3, int(n**0.5)+1, 2):
        if n % i == 0:
            return False
    return True
    
def handle_count_prime(req):
    a, b = req.a, req.b
    client = req.client_id
    count = sum(1 for i in range (a, b+1) if isPrime(i))
    rospy.loginfo(f"Client {client}: Counted {count} primes between {a} and {b}")
    return CountPrimesResponse(prime_count=count, client_id=client)

def prime_server():
    rospy.init_node("prime_server")
    service = rospy.service("count_primes", CountPrimes, handle_count_prime)
    rospy.loginfo("Prime service is ready.")
    rospy.spin()

if __name__ == '__main__':
    prime_server()