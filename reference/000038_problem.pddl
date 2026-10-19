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
        (in-basket red-spray-bottle)
        (on-table roll-of-black-tape)
        (on-table roll-of-green-tape)
        (on-table roll-of-blue-tape)
        (on-table blue-cup-with-yellow-stripe)
        (robot-gripping green-mug)
        (on-table yellow-block)
        (on-table orange-block)
        (on-table blue-block)
        (on-table red-block)
        (on-table green-block)
    )
    (:goal (and ))
)
