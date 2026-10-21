; The order-assembly problem. Write the objects and a goal.
;
; :objects  the items on the table: every item the robot knows, by its name
;           in config/items.yaml (e.g. red-can), all of type item
; :goal     any goal over the domain's predicates, e.g. (in-basket red-can)
;
; The goal console reads this file. It keeps the objects (and adds the items
; added with new-item:), and replaces :init with the current state (at the
; start: every object on the table, the gripper empty) and :goal with the
; goal you type. The rest stays as it is.

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
        ; filled in by the goal console: the current state
        (robot-gripper-empty)
    )

    (:goal
        (and
            (in-basket red-can)
        )
    )
)
