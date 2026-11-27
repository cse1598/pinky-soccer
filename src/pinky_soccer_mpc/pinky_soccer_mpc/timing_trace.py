"""Bounded diagnostics only. No decisions, clocks, threads, commands, or solver calls."""


class TimingTrace:
    def __init__(self):
        self.goal_id=None
        self.slots=self.missed=self.init_missed=self.track_missed=0
        self.job_seq=self.canceled_jobs=0
        self.last_job=self.last_slot=None

    def goal(self,goal_id):
        self.goal_id=str(goal_id)
        self.slots=self.missed=self.init_missed=self.track_missed=0
        self.last_slot=None

    def completed(self,solve_id,goal_id,generation,current_generation,submitted,submitted_sim,due,res,pickup):
        self.job_seq+=1
        canceled=generation!=current_generation
        self.canceled_jobs+=int(canceled)
        started,ended=res.get('job_start_wall'),res.get('job_end_wall')
        self.last_job=dict(seq=self.job_seq,solve_id=solve_id,goal_id=str(goal_id),canceled=canceled,
            generation=generation,current_generation=current_generation,status=res.get('status'),ok=res.get('ok'),
            submitted_wall_s=submitted,pickup_tick_wall_s=pickup,
            submitted_sim_s=submitted_sim,scheduled_due_sim_s=due,ms=res.get('ms'),
            worker_wait_ms=(started-submitted)*1000 if started is not None else None,
            worker_run_ms=(ended-started)*1000 if started is not None and ended is not None else None,
            result_pickup_ms=(pickup-ended)*1000 if ended is not None else None,
            end_to_pickup_ms=(pickup-submitted)*1000,
            acados_ms=sum(a.get('acados_ms',0.) for a in res.get('attempt_statuses') or []))

    def activation(self,now,due,jitter,accepted,had_offer,tracking,pending_id,tick_wall=None):
        self.slots+=1
        self.missed+=int(not accepted)
        self.init_missed+=int(not accepted and not tracking)
        self.track_missed+=int(not accepted and tracking)
        self.last_slot=dict(seq=self.slots,goal_id=self.goal_id,due_sim_s=due,actual_sim_s=now,
            jitter_sim_s=jitter,accepted=bool(accepted),had_offer=bool(had_offer),
            activation_tick_wall_s=tick_wall,
            tracking_before_activation=bool(tracking),pending_solve_id=pending_id)

    def snapshot(self):
        return dict(schema=1,goal_id=self.goal_id,slots=self.slots,missed=self.missed,
            init_missed=self.init_missed,track_missed=self.track_missed,job_seq=self.job_seq,
            canceled_jobs=self.canceled_jobs,last_job=self.last_job,last_slot=self.last_slot)
