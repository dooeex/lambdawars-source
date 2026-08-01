//====== Copyright © Sandern Corporation, All rights reserved. ===========//
//
// Purpose: 
//
// $NoKeywords: $
//=============================================================================//
#include "cbase.h"
#include "unit_airnavigator.h"
#include "unit_locomotion.h"
#include "hl2wars_util_shared.h"

#include "recast/recast_mgr.h"
#include "recast/recast_mesh.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

#ifdef ENABLE_PYTHON
//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
UnitBaseAirNavigator::UnitBaseAirNavigator( boost::python::object outer )
	: UnitBaseNavigator(outer)
{
	m_iTestRouteMask = MASK_NPCSOLID_BRUSHONLY;
	m_bUseSimplifiedRouteBuilding = true;
	m_bTestRouteWorldOnly = true;
	m_fCurrentHeight = m_fDesiredHeight = 0.0f;
	m_bHeightDominator = false;
	m_bHeightRoleInitialized = false;
	m_hHeightPartner = NULL;
}
#endif // ENABLE_PYTHON

//-----------------------------------------------------------------------------
// Purpose: 
//-----------------------------------------------------------------------------
void UnitBaseAirNavigator::Update(UnitAirMoveCommand &MoveCommand)
{
	// Get reported height from locomotion
	m_fCurrentHeight = MoveCommand.height;
	m_fDesiredHeight = MoveCommand.desiredheight;

	bool bIsAtMinDesiredHeight = m_fCurrentHeight >= m_fDesiredHeight;

	BaseClass::Update(MoveCommand);

	// Calculate upmove if needed
	MoveCommand.upmove = 0.0f;

	if (GetPath()->m_iGoalType != GOALTYPE_NONE && GetPath()->GetCurWaypoint())
	{
		bool bCurTargetIsGoal =
			(GetPath()->m_iGoalType == GOALTYPE_TARGETENT ||
			GetPath()->m_iGoalType == GOALTYPE_TARGETENT_INRANGE) &&
			GetPath()->CurWaypointIsGoal();

		float fTargetZ =
			GetPath()->GetCurWaypoint()->GetPos().z +
			(-GetOuter()->CollisionProp()->OBBMins().z);

		UnitBaseAirNavigator *pTargetAirNavigator = nullptr;

		// Check if target is an air unit and determine height roles
		if (bCurTargetIsGoal && GetPath()->m_hTarget)
		{
			CBaseEntity *pTarget = GetPath()->m_hTarget.Get();

			if (pTarget && pTarget->MyUnitPointer())
			{
				pTargetAirNavigator =
					dynamic_cast<UnitBaseAirNavigator*>(
					pTarget->MyUnitPointer()->GetNavigator());

				if (pTargetAirNavigator)
				{
					if (m_hHeightPartner != pTarget)
					{
						m_hHeightPartner = pTarget;

						m_bHeightRoleInitialized = false;
						pTargetAirNavigator->m_bHeightRoleInitialized = false;
					}

					if (!m_bHeightRoleInitialized)
					{
						m_bHeightDominator =
							GetAbsOrigin().z >= pTarget->GetAbsOrigin().z;

						m_bHeightRoleInitialized = true;
					}

					if (!pTargetAirNavigator->m_bHeightRoleInitialized)
					{
						pTargetAirNavigator->m_bHeightDominator =
							!m_bHeightDominator;

						pTargetAirNavigator->m_bHeightRoleInitialized = true;
					}
				}
			}
		}

		// Normal ground target height handling
		if (!pTargetAirNavigator)
		{
			if (m_LastGoalStatus == CHS_CLIMBDEST || !bCurTargetIsGoal)
			{
				fTargetZ += m_fDesiredHeight;
			}
		}

		if (m_LastGoalStatus == CHS_CLIMBDEST || bCurTargetIsGoal)
		{
			float flDelta = fTargetZ - GetAbsOrigin().z;

			// Air target handling
			if (pTargetAirNavigator)
			{
				const float flTolerance = 24.0f;

				// Only the non-dominator adjusts height.
				if (!m_bHeightDominator)
				{
					if (flDelta > flTolerance)
					{
						MoveCommand.upmove = Max(
							-MoveCommand.maxspeed,
							Min(
							flDelta / MoveCommand.interval,
							MoveCommand.maxspeed
							)
							);
					}
				}
				else
				{
					// Dominator never changes altitude because of another air unit.
					MoveCommand.upmove = 0.0f;
				}
			}
			else
			{
				// Normal air navigation (ground targets / waypoints)
				bool bIsManhack = GetOuter()->ClassMatches("unit_manhack");

				if (bIsManhack && bCurTargetIsGoal && GetPath()->m_hTarget)
				{
					CBaseEntity *pTarget = GetPath()->m_hTarget.Get();

					if (pTarget)
					{
						float targetZ =
							pTarget->WorldSpaceCenter().z;

						float delta = targetZ - GetAbsOrigin().z;

						if (delta < -10.0f)
						{
							MoveCommand.upmove = Max(
								-MoveCommand.maxspeed,
								Min(
								delta / MoveCommand.interval,
								MoveCommand.maxspeed
								)
								);
						}
					}
				}
				else if (flDelta > 0.0f)
				{
					MoveCommand.upmove = Max(
						-MoveCommand.maxspeed,
						Min(
						flDelta / MoveCommand.interval,
						MoveCommand.maxspeed
						)
						);
				}
				else if (fTargetZ > GetAbsOrigin().z && bIsAtMinDesiredHeight)
				{
					MoveCommand.upmove = 1.0f;
				}
			}

			// Zero out other movement when climbing
			if (m_LastGoalStatus == CHS_CLIMBDEST)
			{
				MoveCommand.forwardmove = 0.0f;
				MoveCommand.sidemove = 0.0f;
			}
		}
		else if (fTargetZ > GetAbsOrigin().z && bIsAtMinDesiredHeight)
		{
			MoveCommand.upmove = 1.0f;
		}
	}
}

}

//-----------------------------------------------------------------------------
// Purpose: 
//-----------------------------------------------------------------------------
CheckGoalStatus_t UnitBaseAirNavigator::MoveUpdateWaypoint( UnitBaseMoveCommand &MoveCommand )
{
	UnitBaseWaypoint *pCurWaypoint = GetPath()->m_pWaypointHead;
	if( pCurWaypoint->SpecialGoalStatus == CHS_CLIMBDEST )
	{
		// Must be at the right height! The regular check is only 2D.
		if( GetPath()->GetCurWaypoint()->GetPos().z + 4.0f > GetAbsOrigin().z )
			return pCurWaypoint->SpecialGoalStatus;
	}
	return BaseClass::MoveUpdateWaypoint( MoveCommand );
}

//-----------------------------------------------------------------------------
// Purpose: 
//-----------------------------------------------------------------------------
UnitBaseWaypoint *UnitBaseAirNavigator::BuildLocalPath( const Vector &vGoalPos )
{
	if( !m_bUseSimplifiedRouteBuilding || GetBlockedStatus() >= BS_LITTLE )
	{
		// Do a simple trace, always do this for air units
		trace_t tr;

		if( m_bTestRouteWorldOnly )
		{
			CTraceFilterWorldOnly filter;
			UTIL_TraceHull( GetAbsOrigin(), vGoalPos, WorldAlignMins(), WorldAlignMaxs(), m_iTestRouteMask, 
				&filter, &tr);
		}
		else
		{
			CTraceFilterSimple filter( GetOuter(), WARS_COLLISION_GROUP_IGNORE_ALL_UNITS );
			UTIL_TraceHull( GetAbsOrigin(), vGoalPos, WorldAlignMins(), WorldAlignMaxs(), m_iTestRouteMask, 
				&filter, &tr);
		}

		if( tr.DidHit() && (!GetPath()->m_hTarget || !tr.m_pEnt || tr.m_pEnt != GetPath()->m_hTarget) )
			return NULL;

		NavDbgMsg("#%d BuildLocalPath: builded local route\n", GetOuter()->entindex());
		return new UnitBaseWaypoint(vGoalPos);
	}
	else
	{
		NavDbgMsg("#%d BuildLocalPath: builded direct route\n", GetOuter()->entindex());
		return new UnitBaseWaypoint(vGoalPos);
	}
}

//-----------------------------------------------------------------------------
// Purpose: 
//-----------------------------------------------------------------------------
extern ConVar unit_route_navmesh_paths;
UnitBaseWaypoint *UnitBaseAirNavigator::BuildNavAreaPath( UnitBasePath *pPath, const Vector &vGoalPos )
{
	if( !unit_route_navmesh_paths.GetBool() )
		return NULL;

	CRecastMesh *pNavMesh = GetNavMesh();
	if( pNavMesh )
	{
		const Vector &vStart = GetAbsOrigin();
		Vector vStartTestPos( vStart - Vector( 0, 0, m_fCurrentHeight) );

		bool bIsPartial;
		UnitBaseWaypoint *pFoundPath = pNavMesh->FindPath( vStart, vGoalPos, 500.0f, pPath->GetTarget(), &bIsPartial, &vStartTestPos );
		if( pFoundPath )
		{
			if( bIsPartial )
				pPath->m_iFlags |= UNITPATH_FLAGS_PARTIAL;
			return pFoundPath;
		}
	}

	return NULL;
}

//-----------------------------------------------------------------------------
// Purpose: 
//-----------------------------------------------------------------------------
CRecastMesh *UnitBaseAirNavigator::GetNavMesh()
{
	// Make something better, but for now prefer the regular mesh for units flying below 100 units.
	// Mainly intended for manhacks.
	if( m_fCurrentHeight != 0 && m_fCurrentHeight < 100.0f )
	{
		CRecastMesh *pMesh = BaseClass::GetNavMesh();
		if( pMesh )
			return pMesh;
	}

	return RecastMgr().GetMesh( "air" );
}