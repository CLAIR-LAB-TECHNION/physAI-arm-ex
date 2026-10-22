(define (domain order-assembly-domain)
    (:requirements :strips :typing)
    (:types item)
    (:predicates 
        (on-table ?i - item)
        (robot-gripping ?i - item)
        (robot-gripper-empty)
        (in-basket ?i - item)
    )
    (:action pick-up
        :parameters (?i - item)
        :precondition (and
            (on-table ?i)
            (robot-gripper-empty)       
        )
        :effect (and
            (robot-gripping ?i)
            (not (robot-gripper-empty))
            (not (on-table ?i))
        )
    )
    (:action put-on-table
        :parameters (?i - item)
        :precondition (and
            (robot-gripping ?i)
        )
        :effect (and
            (on-table ?i)
            (robot-gripper-empty)
            (not (robot-gripping ?i))
        )
    )
    (:action put-in-basket
        :parameters (?i - item)
        :precondition (and
            (robot-gripping ?i)
        )
        :effect (and
            (in-basket ?i)
            (robot-gripper-empty)
            (not (robot-gripping ?i))
        )
    )
)