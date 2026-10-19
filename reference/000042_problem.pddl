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
        (in-basket red-can)
        (on-table red-spray-bottle)
        (in-basket roll-of-black-tape)
        (in-basket roll-of-green-tape)
        (on-table roll-of-blue-tape)
        (in-basket blue-cup-with-yellow-stripe)
        (in-basket green-mug)
        (on-table yellow-block)
        (in-basket orange-block)
        (in-basket blue-block)
        (in-basket red-block)
        (in-basket green-block)
        (robot-gripper-empty)
    )
    (:goal (and ))
)
