(define (problem cluedo_problem)
(:domain cluedo_domain)
(:objects sherlock_robot -robot
          wp0 -location
          wp1 -location 
          wp2 -location 
          wp3 -location 
          wp4 -location 
          hy -hypotesis

)
(:init 
       (at wp0 sherlock_robot)
       (visited wp0)
       (not_initial_location wp1)
       (not_initial_location wp2)
       (not_initial_location wp3)
       (not_initial_location wp4)
       (not_gripper_positioned)
       (not_visited wp1 )
       (not_visited wp2 )
       (not_visited wp3 )
       (not_visited wp4 )
      )
(:goal(and (end_game))
)
)
