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
        (on-table roll-of-black-tape)
        (on-table roll-of-green-tape)
        (on-table roll-of-blue-tape)
        (on-table blue-cup-with-yellow-stripe)
        (on-table green-mug)
        (in-basket yellow-block)
        (robot-gripping orange-block)
        (on-table blue-block)
        (in-basket red-block)
        (in-basket green-block)
    )
    (:goal (and ))
)
