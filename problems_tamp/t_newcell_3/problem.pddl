(define (problem t_newcell_3) 
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
        (on-table roll-of-green-tape)
        (on-table roll-of-blue-tape)
        (on-table blue-cup-with-yellow-stripe)
        (on-table green-mug)
        (on-table yellow-block)
        (on-table orange-block)
        (on-table blue-block)
        (robot-gripping red-block)
        (on-table green-block)
    )
    (:goal (and (in-basket blue-cup-with-yellow-stripe) (in-basket green-mug)))
)
