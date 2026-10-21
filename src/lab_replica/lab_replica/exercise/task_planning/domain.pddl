; The order-assembly domain: the robot picks items up and puts them on the
; table or into the basket.
;
; The names below are fixed: the robot runs exactly these three actions, each
; with one item as its parameter, and the goal console accepts goals over
; exactly these four predicates. Write the preconditions and the effects.
;
;   (on-table ?i)          ?i stands on the table
;   (in-basket ?i)         ?i is in the basket
;   (robot-gripping ?i)    the gripper holds ?i
;   (robot-gripper-empty)  the gripper holds nothing
;
; The requirements are those of the goals (and, or, not, imply, forall,
; exists over items), so that UPF accepts any goal typed in the console.

(define (domain order-assembly-domain)
    (:requirements :strips :typing :negative-preconditions :disjunctive-preconditions
                   :existential-preconditions :universal-preconditions)
    (:types item)
    (:predicates
        (on-table ?i - item)
        (in-basket ?i - item)
        (robot-gripping ?i - item)
        (robot-gripper-empty)
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
