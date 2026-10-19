(define (problem order-assembly-problem) 
    (:domain order-assembly-domain)
    (:objects
      red-can
      red-spray-bottle
      roll-of-black-tape
      roll-of-green-tape
      roll-of-blue-tape
      blue-cup-with-yellow-stripe
      green-mug
      yellow-block
      orange-block
      blue-block
      red-block
      green-block
      - item
    )
    (:init
        (on-table red-can)
        (on-table red-spray-bottle)
        (in-basket roll-of-black-tape)
        (in-basket roll-of-green-tape)
        (in-basket roll-of-blue-tape)
        (in-basket blue-cup-with-yellow-stripe)
        (on-table green-mug)
        (in-basket yellow-block)
        (on-table orange-block)
        (on-table blue-block)
        (in-basket red-block)
        (on-table green-block)
        (robot-gripper-empty)
    )
    (:goal (and ))
)
